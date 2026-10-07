"""Mission choices, shared System One adapter, and one asynchronous request slot."""
import json
import math
import os
import ssl
from pathlib import Path
from queue import Queue, Empty
from threading import Thread
from time import perf_counter, sleep
from urllib.parse import urlsplit
from urllib.request import Request, build_opener, HTTPRedirectHandler, HTTPSHandler


def criteria(observation):
    choices = {
        "EXPLORE": "Look for samples by following a local survey route.",
        "WAIT": "Pause briefly if sensors are invalid or a transient hazard prevents travel.",
    }
    if len(observation["collected"]) >= 3 or observation["battery_pct"] < 20:
        choices["RETURN"] = "Drive to the landing base and deliver the inventory."
    for sample in observation["candidates"]:
        sid = sample["id"]
        if not sample["inspected"]:
            choices[f"INSPECT_{sid}"] = f"Drive to and inspect observed sample {sid}, distance {sample['distance_m']:.1f} m."
        else:
            choices[f"COLLECT_{sid}"] = f"Drive to and collect inspected sample {sid}, distance {sample['distance_m']:.1f} m."
    return choices


def rules(observation):
    if not observation["sensors_valid"]:
        return "WAIT"
    if len(observation["collected"]) >= 3 or observation["battery_pct"] < 20:
        return "RETURN"
    if observation["candidates"]:
        sample = observation["candidates"][0]
        return ("COLLECT_" if sample["inspected"] else "INSPECT_") + sample["id"]
    return "EXPLORE"


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args):
        return None


class Brain:
    def __init__(self, name, timeout=2):
        self.name, self.timeout = name, timeout
        self.delay = float(os.environ.get("MARS_TEST_DELAY_S", "0"))
        if name in ("rules", "slow"):
            return
        self.model = {"laya": "laya", "kev": "kev-4b", "jev": os.environ.get("JEV_MODEL", "typesafe/jev-1.13")}[name]
        defaults = {"laya": "http://127.0.0.1:8009", "kev": "http://127.0.0.1:8008", "jev": "https://openrouter.ai/api"}
        self.endpoint = os.environ.get(name.upper() + "_BASE_URL", defaults[name]).rstrip("/")
        url = urlsplit(self.endpoint)
        if url.username or url.password or url.query or url.fragment or not url.hostname:
            raise ValueError("Invalid endpoint")
        if name in ("laya", "kev") and url.hostname not in ("127.0.0.1", "localhost", "::1"):
            raise ValueError("Local models require a loopback endpoint")
        if url.scheme != "https" and not (url.scheme == "http" and url.hostname in ("127.0.0.1", "localhost", "::1")):
            raise ValueError("Remote endpoints require HTTPS")
        self.key = os.environ.get("OPENROUTER_API_KEY" if name == "jev" else "LOCAL_MODEL_API_KEY", "")
        if name == "jev" and not self.key:
            raise ValueError("Set OPENROUTER_API_KEY")

    def payload(self, observation):
        # The classifier gets the mission state; navigation geometry stays local.
        state = {k: observation[k] for k in ("battery_pct", "collected", "delivered", "candidates", "sensors_valid", "obstacle", "history")}
        return {"model": self.model, "state": "Mars mission: inspect, collect 3 samples, deliver at base. " + json.dumps(state, separators=(",", ":")),
                "questions": {"mission": {"type": "choice", "instructions": "Choose the next mission action. Inspect observed samples before collecting. Return with all 3 samples. Avoid repeated waiting with valid sensors. Local navigation handles obstacles.", "criteria": criteria(observation)}}}

    def decode(self, raw, observation):
        answer = raw["answers"]["mission"]
        choices = criteria(observation)
        if not isinstance(raw.get("model"), str) or not raw["model"] or (self.name != "jev" and raw["model"] != self.model):
            raise ValueError("Wrong served model")
        probabilities = answer["probabilities"]
        confidence = answer.get("confidence")
        if answer["type"] != "choice" or answer["choice"] not in choices or set(probabilities) != set(choices):
            raise ValueError("Invalid choice contract")
        if any(type(p) not in (int, float) or not math.isfinite(p) or not 0 <= p <= 1 for p in probabilities.values()):
            raise ValueError("Invalid probability")
        if abs(sum(probabilities.values()) - 1) > 0.001 or probabilities[answer["choice"]] + 1e-6 < max(probabilities.values()):
            raise ValueError("Invalid distribution")
        if confidence is not None and (type(confidence) not in (int, float) or not math.isfinite(confidence) or not 0 <= confidence <= 1):
            raise ValueError("Invalid confidence")
        return {"choice": answer["choice"], "confidence": confidence, "probabilities": probabilities, "served_model": raw["model"]}

    def decide(self, observation):
        if self.delay:
            sleep(self.delay)
        if self.name in ("rules", "slow"):
            return {"choice": rules(observation), "confidence": None, "probabilities": None}
        payload = self.payload(observation)
        headers = {"Content-Type": "application/json"}
        if self.key:
            headers["Authorization"] = "Bearer " + self.key
        context = ssl.create_default_context()
        if Path("/etc/ssl/cert.pem").is_file():
            context.load_verify_locations(cafile="/etc/ssl/cert.pem")
        opener = build_opener(NoRedirect(), HTTPSHandler(context=context))
        request = Request(self.endpoint + "/v1/systemone", data=json.dumps(payload).encode(), headers=headers)
        with opener.open(request, timeout=self.timeout) as response:
            raw = json.loads(response.read(2 * 1024 * 1024))
        return {**self.decode(raw, observation), "payload": payload}


class Pending:
    def __init__(self, brain, observation, request_id):
        self.observation, self.request_id = observation, request_id
        self.started = perf_counter()
        self.queue = Queue(maxsize=1)
        self.reported = False
        self.thread = Thread(target=self._run, args=(brain,), daemon=True)
        self.thread.start()

    def _run(self, brain):
        try:
            result = brain.decide(self.observation)
        except Exception as exc:
            # Exception text and HTTP bodies can contain credentials.
            result = {"choice": None, "error": type(exc).__name__}
        self.queue.put((result, perf_counter()))

    def poll(self, simulation_s, revision, deadline_s, max_age_s):
        if self.reported:
            return None
        try:
            result, finished = self.queue.get_nowait()
        except Empty:
            if perf_counter() - self.started < deadline_s:
                return None
            result, finished = {"choice": None}, perf_counter()
        reason = result.get("error")
        if finished - self.started >= deadline_s:
            reason = "deadline_exceeded"
        elif revision != self.observation["revision"]:
            reason = "state_changed"
        elif simulation_s - self.observation["simulation_s"] > max_age_s:
            reason = "stale_observation"
        self.reported = True
        return {"decision": result, "rejection": reason, "latency_ms": (finished - self.started) * 1000}
