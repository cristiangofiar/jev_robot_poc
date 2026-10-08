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
    choices = {"STOP": "Stop when the target has been reached; remain stationary for inspection, collection or delivery. Also brake during a hazard."}
    if observation["sensors_valid"]:
        choices.update({
            "FORWARD": "Drive straight ahead when the target is ahead and still far away, or explore clear space ahead.",
            "TURN_LEFT": "Turn left toward a target on the left, or explore open space to the left. Remain in place.",
            "TURN_RIGHT": "Turn right toward a target on the right, or explore open space to the right. Remain in place.",
        })
        last_movement = next((entry for entry in reversed(observation["history"])
                              if entry.get("accepted") in ("FORWARD", "REVERSE", "TURN_LEFT", "TURN_RIGHT")), {})
        if last_movement.get("result") in ("blocked", "interrupted") and last_movement.get("safety"):
            choices["REVERSE"] = "Drive straight backward to gain clearance after the blocked movement. Check rear space and boundaries; then reobserve, turn and go around the obstacle."
    if observation["sensors_valid"] and observation["goal"]["kind"] != "base":
        choices["RETURN_TO_BASE"] = "Start returning when exploration budget or battery reserve is exhausted. Set base goal; then steer with successive movements."
    for sample in observation["candidates"]:
        if observation["sensors_valid"] and sample["inspect_ready"]:
            choices[f"INSPECT_{sample['id']}"] = "Brake and inspect this visible, aligned, nearby sample after a brief stationary dwell. No approach."
        if observation["sensors_valid"] and sample["collect_ready"]:
            choices[f"COLLECT_{sample['id']}"] = "Brake and collect this inspected, nearby sample after a stationary dwell, subject to Supervisor validation. No approach."
    return choices


def rules(observation):
    """Independent deterministic comparison, constrained to the same movement contract."""
    if not observation["sensors_valid"]:
        return "STOP"
    if observation["return_due"] and observation["goal"]["kind"] != "base":
        return "RETURN_TO_BASE"
    goal = observation["goal"]
    if goal["kind"] == "sample":
        sample = next(c for c in observation["candidates"] if c["id"] == goal["id"])
        if sample["collect_ready"]:
            return f"COLLECT_{sample['id']}"
        if sample["inspect_ready"]:
            return f"INSPECT_{sample['id']}"
        if sample["distance_m"] <= .63:
            if not sample["inspected"] and abs(sample["bearing_deg"]) > 10:
                return "TURN_LEFT" if sample["bearing_deg"] > 0 else "TURN_RIGHT"
            return "STOP"
    if goal["kind"] == "base" and goal["distance_m"] <= .27:
        return "STOP"
    if "rules_navigation" in observation:
        nav = observation["rules_navigation"]
        if nav["mode"] == "spin":
            return "TURN_LEFT" if nav["turn"] > 0 else "TURN_RIGHT"
        if nav["mode"] == "arrived" and goal["kind"] == "sample":
            if abs(goal["bearing_deg"]) > 7:
                return "TURN_LEFT" if goal["bearing_deg"] > 0 else "TURN_RIGHT"
            return "FORWARD"
        if nav["mode"] == "forward":
            if abs(nav["turn"]) > .5:
                return "TURN_LEFT" if nav["turn"] > 0 else "TURN_RIGHT"
            return "FORWARD"
        return "STOP"
    # Standalone probes can exercise the reactive comparison without a Webots map.
    target = observation.get("rules_target") or goal
    error = target.get("bearing_deg", 0)
    if abs(error) > 7:
        return "TURN_LEFT" if error > 0 else "TURN_RIGHT"
    if observation["lidar_m"]["front"] < .65 or observation["safety"] == "boundary_ahead":
        return "TURN_LEFT" if observation["lidar_m"]["left"] >= observation["lidar_m"]["right"] else "TURN_RIGHT"
    return "FORWARD"


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
        # Explicit whitelist: no Supervisor truth, raw scans, rule-controller target or PNG bytes.
        keys = ("battery_pct", "time_remaining_s", "explore_remaining_s", "return_due", "return_reason", "goal", "base",
                "candidates", "collected", "delivered", "sensors_valid", "lidar_m", "motion", "exploration", "progress",
                "history", "safety", "last_safety_intervention", "fallback", "action_duration_s", "stopped_s", "boundary_clearance_m")
        state = {k: observation[k] for k in keys}
        goal = observation["goal"]
        bearing = goal.get("bearing_deg", 0)
        direction = "left" if bearing > 7 else "right" if bearing < -7 else "straight ahead"
        distance = goal.get("distance_m")
        choices = criteria(observation)
        context = f"Active goal: {goal['kind']}. "
        if distance is not None:
            context += f"Target is {direction}, bearing {bearing} degrees, distance {distance} metres. "
            if goal["kind"] == "sample":
                sample = next((c for c in observation["candidates"] if c["id"] == goal["id"]), goal)
                if f"COLLECT_{goal['id']}" in choices:
                    context += f"Inspection confirmed; COLLECT_{goal['id']} is available. "
                elif f"INSPECT_{goal['id']}" in choices:
                    context += f"Sample visible and aligned; INSPECT_{goal['id']} is available. Further approach can lose the camera view. "
                elif distance <= .63 and not sample.get("inspected", False):
                    if not sample.get("visible", False):
                        context += "Nearby sample is NOT visible: INSPECT unavailable. It may be under or behind the front camera. Reposition to restore its view; proximity alone is insufficient. "
                    else:
                        context += "Nearby sample is visible but not inspectable yet. Face it within 18 degrees and recheck eligibility. "
                else:
                    context += "Sample not ready for an operation; check distance, visibility and eligibility. "
            else:
                context += "Base reached; remain stopped for delivery. " if distance <= .27 else "Target not reached yet. "
        if goal["kind"] == "base":
            context += "Return in progress: steer to base and stop there. "
        else:
            context += "Return condition reached: start returning to base. " if observation["return_due"] else "Continue exploration and sample collection. "
        context += "Front obstacle nearby. " if observation["lidar_m"]["front"] < .65 else "Forward space is clear. "
        return {"model": self.model, "state": context + "There are three samples, but their locations are unknown. " + json.dumps(state, separators=(",", ":")),
                "questions": {"mission": {"type": "choice", "instructions":
                    "Choose ONE short movement or eligible operation. Positive bearing is left, negative right. "
                    "Approach samples while keeping them in front of the camera, ideally 0.5-0.63m away. Select INSPECT when available, then COLLECT after inspection; both brake and dwell. Driving over a sample does not collect it. "
                    "If a nearby uninspected sample is invisible, restore separation using available movements, then face it again; turning on top of it may not restore visibility. "
                    "REVERSE is offered only after the last movement was blocked by protection, including a partial movement interrupted by safety. When offered (especially after front_obstacle or spin_clearance), use REVERSE if rear space and boundaries allow. Reobserve after backing away; when clearance is restored, turn toward a clear side and move forward around the obstacle. Do not repeat blocked turns without repositioning. Reobserve after each move; remembered positions may be stale. "
                    "Keep exploring after known samples are collected. Seek unvisited sectors, avoid nearby obstacles and boundaries. "
                    "When return_due, select RETURN_TO_BASE then steer to base with repeated movements; stop at <=0.27m for delivery. "
                    "History separates decisions from motor commands and measured motion; blocked means STOP by safety. "
                    "Forward and turns remain available during hazards; REVERSE requires a blocked last movement. "
                    "No path planner or automatic recovery exists. At the default 0.8s interval, forward or reverse travels about 0.09m and a turn changes yaw about 15deg. Confidence is not proof.",
                    "criteria": choices}}}

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
            status = response.status
            raw = json.loads(response.read(2 * 1024 * 1024))
        return {**self.decode(raw, observation), "raw_response": raw, "http_status": status, "payload": payload}


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

    def poll(self, current, deadline_s):
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
        elif current["revision"] != self.observation["revision"]:
            reason = "state_changed"
        elif math.dist(current["pose"][:2], self.observation["pose"][:2]) > .06 or abs(
                (current["pose"][2] - self.observation["pose"][2] + math.pi) % (2 * math.pi) - math.pi) > .12:
            reason = "pose_changed"
        elif any(abs(current["lidar_m"][key] - self.observation["lidar_m"][key]) > .2 and
                 min(current["lidar_m"][key], self.observation["lidar_m"][key]) < .8 for key in current["lidar_m"]):
            reason = "clearance_changed"
        self.reported = True
        return {"decision": result, "rejection": reason, "latency_ms": (finished - self.started) * 1000}
