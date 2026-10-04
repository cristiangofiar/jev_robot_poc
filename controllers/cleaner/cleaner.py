"""Webots entrypoint. Run with the Create in worlds/apartment.wbt."""

import argparse
import json
import os
import platform
import re
import sys
from dataclasses import asdict, replace
from pathlib import Path
from time import perf_counter
from typing import Any

# Webots starts this script inside controllers/cleaner, not at the project root.
ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controllers.cleaner.actuators import Actuators
from controllers.cleaner.brains.base import Action, Brain, Decision
from controllers.cleaner.brains.rules import RuleBasedBrain
from controllers.cleaner.perception import Perception, require_device
from controllers.cleaner.safety import SafetyLayer
from experiment.config import Config
from experiment.logger import RunLogger


def validate_world(robot: Any, config: Config, root: Path) -> None:
    world = root / "worlds/apartment.wbt"
    if Path(robot.getWorldPath()).resolve() != world.resolve():
        raise ValueError("This iteration only records worlds/apartment.wbt")
    if robot.getTime() != 0:
        raise ValueError("Reset/reload the world before starting a recorded run")
    if robot.getBasicTimeStep() != config.timestep_ms:
        raise ValueError("Configured timestep_ms must match WorldInfo.basicTimeStep")
    seed = re.search(r"^\s*randomSeed\s+(\d+)\s*$", world.read_text(), re.MULTILINE)
    if seed is None or int(seed[1]) != config.seed:
        raise ValueError("Configured seed must match the explicit WorldInfo.randomSeed")


def run(robot: Any, config: Config, root: Path = ROOT, brain: Brain | None = None) -> Path:
    validate_world(robot, config, root)
    actuators = Actuators(robot, config)
    logger = None
    status, error = "simulator_ended", None
    steps, decisions, fallbacks = 0, 0, 0
    contact_detected = None
    simulator_connected = True
    experiment_emitter = None
    wall_origin = perf_counter()

    def wall_ms() -> float:
        return (perf_counter() - wall_origin) * 1000

    try:
        perception = Perception(robot, config.timestep_ms)
        safety = SafetyLayer(config.critical_distance_m, config.safety_release_distance_m,
                             stop_before_contact=config.stop_before_contact)
        brain = brain if brain is not None else RuleBasedBrain(config.stop_distance_m, config.seed, config.decision_interval_ms)
        logger = RunLogger(root, config)
        logger.context.update(model=brain.name, model_version=brain.version)
        if config.scenario_id != "apartment_static":
            experiment_emitter = require_device(robot, "experiment lifecycle")
        hashes = logger.snapshot(root, config)
        logger.write(
            "run_start", parameters=asdict(config), source_sha256=hashes,
            python_version=platform.python_version(),
            webots_version=os.environ.get("WEBOTS_VERSION"),
            world_format_version="R2025a", robot_name=robot.getName(),
            robot_model=robot.getModel(), modality="structured_text",
            available_actions=[action.value for action in Action],
            initial_simulation_time_s=robot.getTime(),
            ground_truth=None,
            ground_truth_file="ground_truth.jsonl" if experiment_emitter is not None else None,
        )
        if experiment_emitter is not None:
            experiment_emitter.send(json.dumps({
                "kind": "run_start", "directory_name": logger.directory.name,
                "context": logger.context, "parameters": asdict(config),
            }, allow_nan=False).encode("utf-8"))
        requested, applied = Action.STOP, Action.STOP
        interval_steps = config.decision_interval_ms // config.timestep_ms
        print(f"Recording baseline to {logger.directory / 'steps.jsonl'}", flush=True)

        while robot.getTime() < config.max_simulation_time_s:
            if robot.step(config.timestep_ms) == -1:
                simulator_connected = False
                break
            state = perception.read()
            observation = perception.observe(state, applied, config.stop_distance_m)
            perception_end_ms = wall_ms()
            if state.contact_detected is not None:
                contact_detected = bool(contact_detected or state.contact_detected)
            decision = None
            decision_start_ms = decision_end_ms = None
            model_input = None
            if steps % interval_steps == 0:
                decisions += 1
                model_input = observation.to_dict()
                decision_start_ms = wall_ms()
                try:
                    decision = brain.decide(observation)
                    if not isinstance(decision, Decision):
                        raise ValueError("Brain must return a typed Decision")
                    # Validate serializability before applying a requested action.
                    json.dumps(asdict(decision), allow_nan=False)
                except Exception as exc:
                    fallbacks += 1
                    decision = Decision(
                        Action.STOP, brain.name,
                        metadata={"fallback": True, "error_type": type(exc).__name__, "error": str(exc)},
                    )
                decision_end_ms = wall_ms()
                decision = replace(decision, latency_ms=decision_end_ms - decision_start_ms)
                requested = decision.action

            safe = safety.apply(requested, state)
            applied = safe.action
            wheel_velocity = actuators.apply(applied)
            actuation_ms = wall_ms()
            logger.write(
                "step", step_index=steps, simulation_time_s=state.simulation_time_s,
                sensors=asdict(state), sensor_observation=observation.to_dict(),
                model_input=model_input, decision_id=decisions,
                decision=asdict(decision) if decision is not None else None,
                requested_action=requested, applied_action=applied,
                safety_reason=safe.reason, safety_override=applied != requested,
                wheel_velocity_rad_s={"left": wheel_velocity[0], "right": wheel_velocity[1]},
                contact_detected=state.contact_detected,
                timing={
                    "event_time_s": None,
                    "perception_time_s": state.simulation_time_s,
                    "perception_end_wall_ms": perception_end_ms,
                    "decision_start_wall_ms": decision_start_ms,
                    "decision_end_wall_ms": decision_end_ms,
                    "actuation_command_time_s": robot.getTime(),
                    "actuation_command_wall_ms": actuation_ms,
                },
                ground_truth=None, frame_ref=None,
                input_tokens=None, output_tokens=None, estimated_cost_usd=None,
            )
            steps += 1
        else:
            status = "duration_reached"
    except BaseException as exc:
        status, error = "error", {"type": type(exc).__name__, "message": str(exc)}
        raise
    finally:
        try:
            actuators.apply(Action.STOP)
            # setVelocity buffers a command. Flush it without advancing physics.
            stop_command_flushed = simulator_connected and robot.step(0) != -1
            if logger is not None:
                logger.write(
                    "run_end", status=status, error=error,
                    simulation_time_s=robot.getTime(), steps=steps,
                    decisions=decisions, fallback_count=fallbacks,
                    bumper_contact_detected=contact_detected, collision=None,
                    outcome="unscored", final_action=Action.STOP,
                    stop_command_flushed=stop_command_flushed,
                )
        finally:
            if logger is not None:
                logger.close()
    return logger.directory


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "experiment/configs/config.json")
    args = parser.parse_args()
    config = Config.load(args.config if args.config.is_absolute() else ROOT / args.config)
    # Only this executable imports Webots; brains and data work offline.
    from controller import Robot
    run(Robot(), config)


if __name__ == "__main__":
    main()
