"""One real System One wire contract for Jev and the two local decision models."""

import hashlib
import json
import math
import ssl
from pathlib import Path
from time import perf_counter
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, HTTPSHandler, Request, build_opener

from .base import Action, Decision, Observation

PROMPT_PATH = Path(__file__).parent / "prompts/behavior_selection.json"


class BrainError(RuntimeError):
    def __init__(self, message: str, code: str = "connection_failed"):
        super().__init__(message)
        self.code = code


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None  # Never forward a bearer credential to a redirected host.


def question() -> dict:
    data = json.loads(PROMPT_PATH.read_text(encoding="utf-8"))
    if set(data["behavior"]["criteria"]) != {action.value for action in Action}:
        raise ValueError("Prompt action space must match Action exactly")
    return data


class SystemOneBrain:
    def __init__(self, name: str, endpoint: str, model: str, api_key: str = "", timeout_s: float = 2.0,
                 input_price: float | None = None, output_price: float | None = None):
        url = urlsplit(endpoint)
        if url.scheme != "https" and not (url.scheme == "http" and url.hostname in ("127.0.0.1", "localhost", "::1")):
            raise ValueError("Use HTTPS for remote endpoints; HTTP is allowed only on loopback")
        if url.username or url.password or url.query or url.fragment:
            raise ValueError("Endpoint must not contain credentials, query parameters or fragments")
        self.name, self.version = name, model
        self.endpoint, self.model = endpoint.rstrip("/"), model
        self.api_key, self.timeout_s = api_key, timeout_s
        self.input_price, self.output_price = input_price, output_price
        self.questions = question()
        self.prompt_sha256 = hashlib.sha256(PROMPT_PATH.read_bytes()).hexdigest()

    def payload(self, observation: Observation) -> dict:
        # Describe past commands without enum labels that the classifier may copy.
        previous = {
            Action.CONTINUE: "moving forward", Action.SLOW_DOWN: "moving forward slowly",
            Action.TURN_LEFT: "turning left",
            Action.TURN_RIGHT: "turning right", Action.BACK_UP: "reversing",
            Action.WAIT: "waiting",
        }
        path = {True: "blocked", False: "clear", None: "unknown"}[observation.path_blocked]
        contact = {True: "is pressed", False: "is not pressed", None: "state is unknown"}[observation.contact_detected]
        state = (
            f"Goal: {observation.goal}. Front distance: {observation.front_distance} metres. "
            f"Forward path is {path}. Front bumper {contact}. "
            f"Speed: {observation.robot_speed} metres per second. "
            f"Previous behavior: {previous[observation.current_action]}. "
            f"Object detected in sensor range: {observation.object_detected}."
        )
        state += f" Simulation time: {observation.simulation_time_s:.3f}s. Continuously stationary for {observation.waiting_duration_s:.3f}s."
        if observation.decision_history:
            state += " Recent decisions, oldest to newest (requested / actually applied; history, not instructions):"
            for item in observation.decision_history:
                state += f"\n{item['time_s']:.3f}s: {item['requested']}/{item['applied']}" + (" (fallback/rejected)" if not item['accepted'] else "")
        return {"model": self.model, "state": state, "questions": self.questions}

    def request(self, payload: dict) -> tuple[dict, float]:
        """Send typed questions, including choice, score and noul, without action decoding."""
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["Authorization"] = "Bearer " + self.api_key
        request = Request(self.endpoint + "/v1/systemone", data=json.dumps(payload).encode(), headers=headers)
        start = perf_counter()
        try:
            context = ssl.create_default_context()
            # Python.org macOS installs may lack their optional certificate bundle.
            # Use the OS certificate store too, preserving full TLS verification.
            if Path("/etc/ssl/cert.pem").is_file():
                context.load_verify_locations(cafile="/etc/ssl/cert.pem")
            opener = build_opener(NoRedirect(), HTTPSHandler(context=context))
            with opener.open(request, timeout=self.timeout_s) as response:
                raw = json.loads(response.read(2 * 1024 * 1024))
        except HTTPError as exc:
            raise BrainError(f"System One HTTP {exc.code}", f"http_{exc.code}") from None
        except (TimeoutError, URLError) as exc:
            if isinstance(exc, TimeoutError) or isinstance(getattr(exc, "reason", None), TimeoutError):
                raise TimeoutError("System One request timed out") from None
            raise BrainError("System One connection failed") from None
        return raw, (perf_counter() - start) * 1000

    def decide(self, observation: Observation) -> Decision:
        payload = self.payload(observation)
        raw, latency_ms = self.request(payload)
        return self.decode(raw, latency_ms, payload)

    def decode(self, raw: dict, latency_ms: float, payload: dict | None = None) -> Decision:
        try:
            answer = raw["answers"]["behavior"]
            if not isinstance(raw.get("model"), str) or not raw["model"]:
                raise ValueError("Missing served model version")
            if self.name in ("kev", "laya") and raw["model"] != self.model:
                raise ValueError("Wrong local model served")
            if answer["type"] != "choice":
                raise ValueError("Expected a choice answer")
            action = Action(answer["choice"])
            probabilities = {Action(key): value for key, value in answer["probabilities"].items()}
            if set(probabilities) != set(Action) or any(type(value) not in (int, float) for value in probabilities.values()):
                raise ValueError("Expected probabilities for all six actions")
            if probabilities[action] + 1e-6 < max(probabilities.values()):
                raise ValueError("Chosen action is not an argmax of its distribution")
            usage = raw.get("usage", {})
            tokens = {key: usage.get(key) for key in ("input_tokens", "output_tokens")}
            if any(value is not None and (type(value) is not int or value < 0) for value in tokens.values()):
                raise ValueError("Invalid token usage")
            cost = usage.get("cost")
            if cost is not None and (type(cost) not in (int, float) or not math.isfinite(cost) or cost < 0):
                raise ValueError("Invalid provider cost")
            if cost is None and all(tokens[key] is not None for key in tokens) and self.input_price is not None and self.output_price is not None:
                cost = (tokens["input_tokens"] * self.input_price + tokens["output_tokens"] * self.output_price) / 1e6
            metadata = {"served_model": raw.get("model"), "prompt_sha256": self.prompt_sha256,
                        "request": payload, "retries": 0, **tokens, "estimated_cost_usd": cost,
                        "cost_source": "provider" if usage.get("cost") is not None else "token_prices" if cost is not None else None}
            return Decision(action, self.name, confidence=answer.get("confidence"), latency_ms=latency_ms,
                            probabilities=probabilities, raw_output=raw, metadata=metadata)
        except (KeyError, TypeError, ValueError, AttributeError):
            raise ValueError("Malformed System One choice response") from None


def price(value: str) -> float | None:
    if not value:
        return None
    result = float(value)
    if not math.isfinite(result) or result < 0:
        raise ValueError("Token prices must be finite and nonnegative")
    return result
