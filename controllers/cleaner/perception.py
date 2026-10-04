"""The only sensor-reading boundary; no scenario ground truth enters a Brain."""

from dataclasses import dataclass
from math import isfinite
from typing import Any

from controllers.cleaner.brains.base import Action, Observation


def finite_nonnegative(value: float) -> float | None:
    return float(value) if isfinite(value) and value >= 0 else None


def require_device(robot: Any, name: str) -> Any:
    device = robot.getDevice(name)
    if device is None:
        raise RuntimeError(f"Missing Webots device: {name!r}; check apartment.wbt / Create.proto")
    return device


@dataclass(frozen=True)
class SensorState:
    simulation_time_s: float
    front_distance_m: float | None
    robot_speed_m_s: float | None
    position_m: tuple[float, float, float] | None
    bumper_left: bool | None
    bumper_right: bool | None

    @property
    def contact_detected(self) -> bool | None:
        if self.bumper_left or self.bumper_right:
            return True
        if self.bumper_left is None or self.bumper_right is None:
            return None
        return False


class Perception:
    def __init__(self, robot: Any, timestep_ms: int):
        self.robot = robot
        self.front = require_device(robot, "front distance")
        self.position = require_device(robot, "benchmark gps")
        self.left = require_device(robot, "bumper_left")
        self.right = require_device(robot, "bumper_right")
        for sensor in (self.front, self.position, self.left, self.right):
            sensor.enable(timestep_ms)
        self.range_m = self.front.getMaxValue()

    def read(self) -> SensorState:
        position = tuple(self.position.getValues())
        left, right = self.left.getValue(), self.right.getValue()
        return SensorState(
            simulation_time_s=self.robot.getTime(),
            front_distance_m=finite_nonnegative(self.front.getValue()),
            robot_speed_m_s=finite_nonnegative(self.position.getSpeed()),
            position_m=position if all(isfinite(x) for x in position) else None,
            bumper_left=bool(left) if isfinite(left) else None,
            bumper_right=bool(right) if isfinite(right) else None,
        )

    def observe(self, state: SensorState, action: Action, stop_distance_m: float) -> Observation:
        distance = state.front_distance_m
        return Observation(
            robot_speed=state.robot_speed_m_s,
            front_distance=distance,
            object_detected=None if distance is None else distance < self.range_m,
            path_blocked=None if distance is None else distance < stop_distance_m,
            current_action=action,
            contact_detected=state.contact_detected,
        )

