"""Small JSON configuration, using only the standard library."""

import json
from dataclasses import dataclass
from math import isfinite
from pathlib import Path


@dataclass(frozen=True)
class Config:
    experiment_id: str = "baseline_v1"
    scenario_id: str = "apartment_static"
    brain: str = "rules"
    seed: int = 0
    timestep_ms: int = 16
    decision_interval_ms: int = 64
    forward_speed_m_s: float = 0.20
    turn_wheel_speed_rad_s: float = 2.0
    wheel_radius_m: float = 0.031
    stop_distance_m: float = 0.45
    critical_distance_m: float = 0.12
    safety_release_distance_m: float = 0.16
    max_simulation_time_s: float = 20.0
    results_dir: str = "results/raw"
    stop_before_contact: bool = False
    decision_timeout_s: float = 3.0
    max_decision_age_s: float = 2.0
    batch_mode: bool = False

    def __post_init__(self) -> None:
        if self.brain not in ("rules", "threshold", "kev", "laya", "jev"):
            raise ValueError("Unknown brain")
        if type(self.batch_mode) is not bool:
            raise ValueError("batch_mode must be a bool")
        if type(self.stop_before_contact) is not bool:
            raise ValueError("stop_before_contact must be a bool")
        for name in ("experiment_id", "scenario_id", "results_dir"):
            if not isinstance(getattr(self, name), str) or not getattr(self, name):
                raise ValueError(f"{name} must be a nonempty string")
        if type(self.seed) is not int or not 0 <= self.seed < 2**31:
            raise ValueError("seed must be a nonnegative 32-bit integer")
        for name in ("timestep_ms", "decision_interval_ms"):
            if type(getattr(self, name)) is not int or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if self.decision_interval_ms % self.timestep_ms:
            raise ValueError("decision_interval_ms must be a multiple of timestep_ms")
        for name in (
            "forward_speed_m_s", "turn_wheel_speed_rad_s", "wheel_radius_m",
            "stop_distance_m", "critical_distance_m", "safety_release_distance_m",
            "max_simulation_time_s",
            "decision_timeout_s", "max_decision_age_s",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not self.critical_distance_m < self.safety_release_distance_m < self.stop_distance_m:
            raise ValueError("Require critical < safety release < baseline stop distance")

    @classmethod
    def load(cls, path: Path) -> "Config":
        return cls(**json.loads(path.read_text(encoding="utf-8")))
