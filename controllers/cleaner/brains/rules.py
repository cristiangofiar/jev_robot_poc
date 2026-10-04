"""Advance until contact, then briefly retreat and turn in a seeded random direction."""

from math import ceil
from random import Random

from .base import Action, Decision, Observation


class ThresholdBrain:
    """The original simple, deterministic WAIT/CONTINUE baseline."""
    name, version = "threshold", "1"

    def __init__(self, stop_distance_m: float):
        self.stop_distance_m = stop_distance_m

    def decide(self, observation: Observation) -> Decision:
        distance = observation.front_distance
        return Decision(Action.WAIT if distance is None or distance < self.stop_distance_m else Action.CONTINUE, self.name)


class RuleBasedBrain:
    name = "rules"
    version = "3"

    def __init__(self, stop_distance_m: float, seed: int = 0, decision_interval_ms: int = 64):
        self.stop_distance_m = stop_distance_m
        self.random = Random(seed)
        self.interval_s = decision_interval_ms / 1000
        self.action = Action.CONTINUE
        self.remaining = 0

    def _ticks(self, seconds: float) -> int:
        return max(1, ceil(seconds / self.interval_s))

    def _turn(self) -> None:
        self.action = self.random.choice((Action.TURN_LEFT, Action.TURN_RIGHT))
        self.remaining = self._ticks(self.random.uniform(1, 3))

    def decide(self, observation: Observation) -> Decision:
        distance = observation.front_distance
        if distance is None:
            return Decision(Action.WAIT, self.name)

        if self.action == Action.BACK_UP:
            if self.remaining == 0:
                self._turn()
        elif observation.contact_detected:
            self.action = Action.BACK_UP
            self.remaining = self._ticks(0.5)
        elif self.action in (Action.TURN_LEFT, Action.TURN_RIGHT) and self.remaining == 0:
            self.action = Action.CONTINUE

        if self.remaining > 0:
            self.remaining -= 1
        return Decision(self.action, self.name)
