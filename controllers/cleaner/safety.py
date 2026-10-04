"""Independent hard stop, evaluated every physical timestep."""

from dataclasses import dataclass

from controllers.cleaner.brains.base import Action
from controllers.cleaner.perception import SensorState


@dataclass(frozen=True)
class SafetyResult:
    action: Action
    reason: str | None


class SafetyLayer:
    def __init__(self, critical_distance_m: float, release_distance_m: float, stop_before_contact: bool = True):
        self.critical_distance_m = critical_distance_m
        self.release_distance_m = release_distance_m
        self.latched = False
        self.stop_before_contact = stop_before_contact

    def apply(self, requested: Action, state: SensorState) -> SafetyResult:
        distance = state.front_distance_m
        if distance is None or state.bumper_left is None or state.bumper_right is None:
            self.latched = True
            return SafetyResult(Action.STOP, "invalid_safety_sensor")
        # Front sensors may block forward motion, but must allow retreat.
        if requested == Action.BACK_UP:
            return SafetyResult(requested, None)
        if state.contact_detected:
            self.latched = True
            return SafetyResult(Action.STOP, "bumper_contact")
        if requested in (Action.TURN_LEFT, Action.TURN_RIGHT):
            return SafetyResult(requested, None)
        if not self.stop_before_contact:
            self.latched = False
            return SafetyResult(requested, None)
        if distance < self.critical_distance_m:
            self.latched = True
            return SafetyResult(Action.STOP, "critical_front_distance")
        if self.latched and distance < self.release_distance_m:
            return SafetyResult(Action.STOP, "safety_hysteresis")
        self.latched = False
        return SafetyResult(requested, None)

