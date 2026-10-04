"""Webots Supervisor: one falling box and a separate ground truth stream."""

import argparse
import json
import re
import sys
from dataclasses import asdict
from math import dist
from pathlib import Path
from time import monotonic
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controllers.cleaner.cleaner import validate_world
from controllers.cleaner.perception import require_device
from experiment.config import Config
from experiment.logger import GroundTruthLogger
from experiment.scenario import FallingBoxScenario


def attach_run(root: Path, config: Config, message: dict[str, Any]) -> GroundTruthLogger:
    """Accept only a matching, already-recorded run under the configured results directory."""
    if not isinstance(message, dict) or message.get("kind") != "run_start" or message.get("parameters") != asdict(config):
        raise ValueError("Cleaner and Supervisor must use the same effective configuration")
    name = message.get("directory_name")
    if not isinstance(name, str) or Path(name).name != name or name in (".", ".."):
        raise ValueError("Invalid run directory name")
    results = (root / config.results_dir).resolve()
    directory = (results / name).resolve()
    if directory.parent != results:
        raise ValueError("Run directory escapes configured results directory")
    context = message.get("context", {})
    context_keys = {"schema_version", "experiment_id", "run_id", "scenario_id", "seed", "model", "model_version"}
    if (not isinstance(context, dict) or set(context) != context_keys
            or not isinstance(context.get("run_id"), str)
            or not re.fullmatch(r"[0-9a-f]{32}", context["run_id"])):
        raise ValueError("Invalid run context")
    with (directory / "steps.jsonl").open(encoding="utf-8") as file:
        start = json.loads(file.readline())
    if start.get("record_type") != "run_start" or any(start.get(key) != value for key, value in context.items()):
        raise ValueError("Lifecycle message does not match the recorded run")
    if json.loads((directory / "config.json").read_text()) != asdict(config):
        raise ValueError("Recorded configuration differs from Supervisor configuration")
    return GroundTruthLogger(directory, context)


def shared_contact_points(robot_points: list, box_points: list) -> list[list[float]]:
    """Identify pair contact by matching their world-space contact points (1 µm tolerance)."""
    return [list(point.point) for point in box_points
            if any(dist(point.point, other.point) <= 1e-6 for other in robot_points)]


def finish_batch(supervisor, config, logger):
    """Quit only after both writers finished; the runner handles abnormal termination."""
    deadline = monotonic() + 10
    complete = False
    while logger is not None and monotonic() < deadline:
        # Last record is small, even when sensor/decision records are much larger.
        with (logger.directory / "steps.jsonl").open("rb") as file:
            file.seek(max(0, file.seek(0, 2) - 8192))
            lines = file.read().splitlines()
        try:
            end = json.loads(lines[-1]) if lines else {}
        except json.JSONDecodeError:
            end = {}
        if end.get("record_type") == "run_end" and end.get("run_id") == logger.run_id:
            complete = end.get("status") == "duration_reached" and end.get("stop_command_flushed") is True
            break
        if supervisor.step(config.timestep_ms) == -1:
            break
    supervisor.simulationQuit(0 if complete else 1)


def run_static_batch(supervisor, config, root):
    receiver = require_device(supervisor, "experiment lifecycle")
    receiver.enable(config.timestep_ms)
    logger = None
    status = "duration_reached"
    while supervisor.getTime() < config.max_simulation_time_s:
        if supervisor.step(config.timestep_ms) == -1:
            status = "simulator_ended"
            break
        while receiver.getQueueLength():
            message = json.loads(receiver.getString())
            receiver.nextPacket()
            if logger is not None:
                raise ValueError("Duplicate cleaner metadata")
            logger = attach_run(root, config, message)
            logger.write("scenario_start", scenario=None, simulation_time_s=supervisor.getTime())
    if logger:
        logger.write("scenario_end", status=status, simulation_time_s=supervisor.getTime(),
                     event_released=False, outcome="unscored", collision_with_spawned_object=None)
        logger.close()
    finish_batch(supervisor, config, logger)
    return logger.directory if logger else None


def run(supervisor: Any, config: Config, root: Path = ROOT) -> Path | None:
    validate_world(supervisor, config, root)
    scenario = FallingBoxScenario.load(root, config)
    if scenario is None:
        if config.batch_mode:
            return run_static_batch(supervisor, config, root)
        return None  # Static apartment: do not manipulate the world or require metadata.
    robot = supervisor.getFromDef("CLEANER")
    if robot is None:
        raise RuntimeError("Missing DEF CLEANER in apartment.wbt")
    receiver = require_device(supervisor, "experiment lifecycle")
    receiver.enable(config.timestep_ms)
    children = supervisor.getRoot().getField("children")
    logger = None
    box = None
    connected = True
    status, error = "simulator_ended", None
    released = False
    collision = False
    minimum_center_distance = None

    try:
        stale_box = supervisor.getFromDef("EXPERIMENT_BOX")
        if stale_box is not None:
            stale_box.remove()
        # Queue the same starting pose before the first physics step for every brain.
        robot.getField("translation").setSFVec3f(list(scenario.initial_translation_m))
        robot.getField("rotation").setSFRotation([0, 0, 1, scenario.initial_yaw_rad])
        robot.resetPhysics()

        while supervisor.getTime() < config.max_simulation_time_s:
            if supervisor.step(config.timestep_ms) == -1:
                connected = False
                break
            now = supervisor.getTime()
            while receiver.getQueueLength():
                message = json.loads(receiver.getString())
                receiver.nextPacket()
                if logger is not None:
                    raise ValueError("Duplicate cleaner run metadata; reload the world")
                logger = attach_run(root, config, message)
                recorded_scenario = json.loads(
                    (logger.directory / "snapshot/experiment/scenarios" / f"{config.scenario_id}.json").read_text()
                )
                if recorded_scenario != json.loads(json.dumps(asdict(scenario))):
                    raise ValueError("Supervisor scenario differs from the cleaner source snapshot")
                logger.write(
                    "scenario_start", scenario=asdict(scenario), simulation_time_s=now,
                    initial_translation_m=scenario.initial_translation_m,
                    nominal_release_position_m=scenario.release_position(config.forward_speed_m_s),
                    nominal_ground_truth=scenario.nominal_ground_truth(config.forward_speed_m_s),
                    collision_method="shared_world_contact_points_with_1e-6_m_tolerance",
                )
                print(f"Ground truth: {logger.directory / 'ground_truth.jsonl'}", flush=True)

            if now + 1e-9 >= scenario.release_time_s and box is None:
                if logger is None:
                    raise TimeoutError("No matching cleaner run metadata before the scheduled event")
                children.importMFNodeFromString(-1, scenario.box_node(config.forward_speed_m_s))
                box = supervisor.getFromDef("EXPERIMENT_BOX")
                if box is None:
                    raise RuntimeError("Webots failed to import EXPERIMENT_BOX")
                released = True
                logger.write(
                    "event_released", scheduled_time_s=scenario.release_time_s,
                    release_command_time_s=now,
                    first_physics_step_after_release_s=now + config.timestep_ms / 1000,
                    release_position_m=list(box.getPosition()),
                    actual_robot_position_m=list(robot.getPosition()),
                    ideal_action=scenario.ideal_action,
                )

            if box is not None:
                robot_position, box_position = list(robot.getPosition()), list(box.getPosition())
                robot_contacts = robot.getContactPoints(True)
                box_contacts = box.getContactPoints(True)
                pair_contacts = shared_contact_points(robot_contacts, box_contacts)
                center_distance = dist(robot_position, box_position)
                minimum_center_distance = min(
                    center_distance, minimum_center_distance if minimum_center_distance is not None else center_distance
                )
                collision = collision or bool(pair_contacts)
                logger.write(
                    "trajectory", simulation_time_s=now,
                    robot_position_m=robot_position, robot_orientation=list(robot.getOrientation()),
                    robot_velocity=list(robot.getVelocity()),
                    object_position_m=box_position, object_orientation=list(box.getOrientation()),
                    object_velocity=list(box.getVelocity()), center_distance_m=center_distance,
                    robot_contact_points_m=[list(point.point) for point in robot_contacts],
                    object_contact_points_m=[list(point.point) for point in box_contacts],
                    pair_contact_points_m=pair_contacts,
                    collision_with_spawned_object=bool(pair_contacts),
                )
        else:
            status = "duration_reached"
    except BaseException as exc:
        status, error = "error", {"type": type(exc).__name__, "message": str(exc)}
        raise
    finally:
        try:
            removed = False
            if box is not None and connected:
                box.remove()
                removed = supervisor.step(0) != -1
            if logger is not None:
                outcome = "incomplete"
                if status == "duration_reached" and released:
                    outcome = "collision" if collision else "no_object_contact_observed"
                logger.write(
                    "scenario_end", simulation_time_s=supervisor.getTime(), status=status,
                    error=error, event_released=released, outcome=outcome,
                    collision_with_spawned_object=collision if released else None,
                    minimum_center_distance_m=minimum_center_distance,
                    object_removed=removed,
                )
        finally:
            if logger is not None:
                logger.close()
    if config.batch_mode:
        finish_batch(supervisor, config, logger)
    return logger.directory if logger is not None else None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "experiment/configs/config.json")
    args = parser.parse_args()
    config = Config.load(args.config if args.config.is_absolute() else ROOT / args.config)
    from controller import Supervisor
    supervisor = Supervisor()
    try:
        run(supervisor, config)
    except BaseException:
        if config.batch_mode:
            supervisor.simulationQuit(1)
        raise


if __name__ == "__main__":
    main()
