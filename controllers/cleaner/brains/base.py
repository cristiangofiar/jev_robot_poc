"""Simulator-independent input/output contract; all distances are in metres."""

from dataclasses import asdict, dataclass, field
from enum import Enum
from math import isfinite
from typing import Any, Protocol


class Action(str, Enum):
    CONTINUE = "CONTINUE"
    SLOW_DOWN = "SLOW_DOWN"
    STOP = "STOP"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    BACK_UP = "BACK_UP"
    WAIT = "WAIT"
    REPLAN = "REPLAN"


@dataclass(frozen=True)
class Observation:
    robot_speed: float | None
    front_distance: float | None
    object_detected: bool | None
    path_blocked: bool | None
    current_action: Action
    goal: str = "continue cleaning"
    contact_detected: bool | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.current_action, Action):
            raise ValueError("current_action must be an Action")
        for value in (self.robot_speed, self.front_distance):
            if value is not None and (
                isinstance(value, bool) or not isfinite(value) or value < 0
            ):
                raise ValueError("measurements must be finite, nonnegative or None")
        for value in (self.object_detected, self.path_blocked, self.contact_detected):
            if value is not None and not isinstance(value, bool):
                raise ValueError("perception flags must be bool or None")
        if not isinstance(self.goal, str):
            raise ValueError("goal must be a string")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Observation":
        return cls(**{**data, "current_action": Action(data["current_action"])})


@dataclass(frozen=True)
class Decision:
    action: Action
    model_name: str
    confidence: float | None = None
    latency_ms: float = 0.0
    raw_output: Any = None
    probabilities: dict[Action, float] | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.action, Action):
            raise ValueError("decision action must be an Action")
        if not isfinite(self.latency_ms) or self.latency_ms < 0:
            raise ValueError("latency_ms must be finite and nonnegative")
        if self.confidence is not None and (
            not isfinite(self.confidence) or not 0 <= self.confidence <= 1
        ):
            raise ValueError("confidence must be in [0, 1] or None")
        if self.probabilities is not None:
            if any(
                not isinstance(action, Action) or not isfinite(p) or not 0 <= p <= 1
                for action, p in self.probabilities.items()
            ) or abs(sum(self.probabilities.values()) - 1) > 1e-6:
                raise ValueError("probabilities must be an Action distribution")


class Brain(Protocol):
    name: str
    version: str

    def decide(self, observation: Observation) -> Decision: ...

