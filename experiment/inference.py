"""At most one inference in flight; physics never waits for a network response."""

import json
from dataclasses import asdict, replace
from threading import Thread
from time import perf_counter

from controllers.cleaner.brains.base import Action, Brain, Decision, Observation
from controllers.cleaner.brains.system_one import BrainError


def infer(brain: Brain, observation: Observation) -> Decision:
    start = perf_counter()
    try:
        decision = brain.decide(observation)
        if not isinstance(decision, Decision):
            raise ValueError("Brain must return a typed Decision")
        json.dumps(asdict(decision), allow_nan=False)
    except Exception as exc:
        # External exceptions may contain credentials; record only their class.
        metadata = {"fallback": True, "error_type": type(exc).__name__}
        if isinstance(exc, BrainError):
            metadata["error_code"] = exc.code
        decision = Decision(Action.WAIT, brain.name, metadata=metadata)
    return replace(decision, latency_ms=(perf_counter() - start) * 1000)


class PendingInference:
    def __init__(self, brain: Brain, observation: Observation, simulation_time_s: float, decision_id: int):
        self.observation = observation
        self.model_name = brain.name
        self.simulation_time_s = simulation_time_s
        self.decision_id = decision_id
        self.started = perf_counter()
        self.result = None
        self.finished = None
        self.expired = False
        self.thread = Thread(target=self._run, args=(brain,), daemon=True)
        self.thread.start()

    def _run(self, brain: Brain) -> None:
        self.result = infer(brain, self.observation)
        self.finished = perf_counter()

    def poll(self, now_simulation_s: float, timeout_s: float, maximum_age_s: float) -> tuple[Decision, str | None] | None:
        """Return (decision, rejection_reason), once. A late result is never applied."""
        if self.expired:
            return None
        elapsed = perf_counter() - self.started
        if self.thread.is_alive() and elapsed < timeout_s:
            return None
        reason = None
        if (self.finished - self.started if self.finished is not None else elapsed) >= timeout_s:
            reason = "deadline_exceeded"
        elif now_simulation_s - self.simulation_time_s > maximum_age_s:
            reason = "stale_observation"
        self.expired = True
        if reason:
            return Decision(Action.WAIT, self.model_name, latency_ms=elapsed * 1000,
                            metadata={"fallback": True, "error_type": reason,
                                      "rejected_decision": asdict(self.result) if self.result else None}), reason
        return self.result, None
