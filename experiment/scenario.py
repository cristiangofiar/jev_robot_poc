"""A single fixed falling-box event, independent of any model's actual trajectory."""

import json
import re
from dataclasses import dataclass
from math import cos, isfinite, sin, sqrt
from pathlib import Path

from controllers.cleaner.brains.base import Action
from experiment.config import Config


@dataclass(frozen=True)
class FallingBoxScenario:
    scenario_id: str
    initial_translation_m: tuple[float, float, float]
    initial_yaw_rad: float
    release_time_s: float
    forward_offset_m: float
    lateral_offset_m: float
    release_center_height_m: float
    box_size_m: tuple[float, float, float]
    box_mass_kg: float
    ideal_action: Action

    def __post_init__(self) -> None:
        if not re.fullmatch(r"[a-z][a-z0-9_]*", self.scenario_id):
            raise ValueError("Invalid scenario_id")
        for vector in (self.initial_translation_m, self.box_size_m):
            if len(vector) != 3 or any(type(x) not in (int, float) or not isfinite(x) for x in vector):
                raise ValueError("Scenario vectors must have three finite numbers")
        for value in (
            self.initial_yaw_rad, self.release_time_s, self.forward_offset_m,
            self.lateral_offset_m, self.release_center_height_m, self.box_mass_kg,
        ):
            if type(value) not in (int, float) or not isfinite(value):
                raise ValueError("Scenario parameters must be finite numbers")
        if self.release_time_s <= 0 or self.forward_offset_m <= 0 or self.box_mass_kg <= 0:
            raise ValueError("Release time, forward offset and mass must be positive")
        if min(self.box_size_m) <= 0 or self.release_center_height_m <= self.box_size_m[2] / 2:
            raise ValueError("Box must have positive size and start above the floor")
        if not isinstance(self.ideal_action, Action):
            raise ValueError("ideal_action must be an Action")

    @classmethod
    def load(cls, root: Path, config: Config) -> "FallingBoxScenario | None":
        if config.scenario_id == "apartment_static":
            return None
        if not re.fullmatch(r"[a-z][a-z0-9_]*", config.scenario_id):
            raise ValueError("Invalid scenario_id")
        path = root / "experiment/scenarios" / f"{config.scenario_id}.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        data["initial_translation_m"] = tuple(data["initial_translation_m"])
        data["box_size_m"] = tuple(data["box_size_m"])
        data["ideal_action"] = Action(data["ideal_action"])
        scenario = cls(**data)
        if scenario.scenario_id != config.scenario_id:
            raise ValueError("Scenario file and configured scenario_id differ")
        if scenario.release_time_s + config.timestep_ms / 1000 >= config.max_simulation_time_s:
            raise ValueError("Run must include at least one physics step after release")
        return scenario

    def release_position(self, speed_m_s: float) -> tuple[float, float, float]:
        # The reference robot moves at nominal speed, regardless of brain actions.
        forward = speed_m_s * self.release_time_s + self.forward_offset_m
        c, s = cos(self.initial_yaw_rad), sin(self.initial_yaw_rad)
        x, y, _ = self.initial_translation_m
        return (x + c * forward - s * self.lateral_offset_m,
                y + s * forward + c * self.lateral_offset_m,
                self.release_center_height_m)

    def nominal_ground_truth(self, speed_m_s: float) -> dict[str, str | float | bool]:
        # Analytic estimate only: free fall, then a stationary box on z=0.
        fall_time = sqrt(2 * (self.release_center_height_m - self.box_size_m[2] / 2) / 9.81)
        horizontal_entry = (self.forward_offset_m - self.box_size_m[0] / 2 - 0.1675) / speed_m_s
        intersects_lane = abs(self.lateral_offset_m) < self.box_size_m[1] / 2 + 0.1675
        return {
            "assumptions": "free_fall_then_static_box; constant_nominal_robot_speed; upright_Create_body",
            "estimated_fall_time_s": fall_time,
            "nominal_body_entry_after_event_s": max(0.0, horizontal_entry),
            "nominal_collision_expected": intersects_lane and horizontal_entry >= fall_time,
            "ideal_action": self.ideal_action.value,
        }

    def box_node(self, speed_m_s: float) -> str:
        position = " ".join(map(str, self.release_position(speed_m_s)))
        size = " ".join(map(str, self.box_size_m))
        return f'''DEF EXPERIMENT_BOX Solid {{
  translation {position}
  rotation 0 0 1 {self.initial_yaw_rad}
  name "experiment falling box"
  children [ Shape {{
    appearance PBRAppearance {{ baseColor 0.65 0.35 0.12 roughness 1 metalness 0 }}
    geometry Box {{ size {size} }}
  }} ]
  boundingObject Box {{ size {size} }}
  physics Physics {{ density -1 mass {self.box_mass_kg} }}
}}'''
