"""Deterministic action-to-wheel mapping; velocities are in rad/s."""

from typing import Any

from controllers.cleaner.brains.base import Action
from controllers.cleaner.perception import require_device
from experiment.config import Config


class Actuators:
    def __init__(self, robot: Any, config: Config):
        self.left = require_device(robot, "left wheel motor")
        self.right = require_device(robot, "right wheel motor")
        forward = config.forward_speed_m_s / config.wheel_radius_m
        turn = config.turn_wheel_speed_rad_s
        limit = min(self.left.getMaxVelocity(), self.right.getMaxVelocity())
        if max(forward, turn) > limit:
            raise ValueError("Configured wheel velocities exceed Create motor limits")
        self.commands = {
            Action.CONTINUE: (forward, forward),
            Action.SLOW_DOWN: (forward / 2, forward / 2),
            Action.TURN_LEFT: (-turn, turn),
            Action.TURN_RIGHT: (turn, -turn),
            Action.BACK_UP: (-forward / 2, -forward / 2),
            Action.WAIT: (0.0, 0.0),
        }
        for motor in (self.left, self.right):
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

    def apply(self, action: Action) -> tuple[float, float]:
        left, right = self.commands[action]
        self.left.setVelocity(left)
        self.right.setVelocity(right)
        return left, right
