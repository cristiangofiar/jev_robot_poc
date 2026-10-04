"""Seeded random wandering with a short retreat and turn near obstacles."""

from math import ceil
from random import Random

from .base import Action, Decision, Observation


class RuleBasedBrain:
    name = "rules"
    version = "2"

    def __init__(self, stop_distance_m: float, seed: int = 0, decision_interval_ms: int = 64):
        self.stop_distance_m = stop_distance_m
        self.random = Random(seed)
        self.interval_s = decision_interval_ms / 1000
        self.action = Action.CONTINUE
        self.remaining = self._ticks(self.random.uniform(3, 6))

    def _ticks(self, seconds: float) -> int:
        return max(1, ceil(seconds / self.interval_s))

    def _turn(self) -> None:
        self.action = self.random.choice((Action.TURN_LEFT, Action.TURN_RIGHT))
        self.remaining = self._ticks(self.random.uniform(1, 3))

    def decide(self, observation: Observation) -> Decision:
        distance = observation.front_distance
        if distance is None:
            return Decision(Action.STOP, self.name)

        blocked = distance < self.stop_distance_m
        if self.action == Action.BACK_UP:
            if self.remaining == 0:
                self._turn()
        elif observation.contact_detected or (self.action == Action.CONTINUE and blocked):
            self.action = Action.BACK_UP
            self.remaining = self._ticks(0.5)
        elif self.remaining == 0:
            if self.action == Action.CONTINUE or blocked:
                self._turn()
            else:
                self.action = Action.CONTINUE
                self.remaining = self._ticks(self.random.uniform(3, 6))

        self.remaining -= 1
        return Decision(self.action, self.name)
