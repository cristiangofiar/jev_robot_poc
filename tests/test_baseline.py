"""Checks the experiment boundary without importing Webots or making API calls."""

import hashlib
import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from controllers.cleaner.brains.base import Action, Decision, Observation
from controllers.cleaner.brains.rules import RuleBasedBrain
from controllers.cleaner.cleaner import ROOT, run, validate_world
from controllers.cleaner.perception import SensorState
from controllers.cleaner.safety import SafetyLayer
from experiment.config import Config
from experiment.logger import RunLogger


class FakeRobot:
    """Only sensor samples and the Webots calls used by the controller."""

    def __init__(self, distances: list[float]):
        self.distances = distances
        self.index = -1
        self.time = 0.0
        self.flushes = 0
        self.devices = {
            "left wheel motor": Mock(getMaxVelocity=Mock(return_value=16.129)),
            "right wheel motor": Mock(getMaxVelocity=Mock(return_value=16.129)),
            "front distance": SimpleNamespace(
                enable=Mock(), getMaxValue=lambda: 2.0,
                getValue=lambda: self.distances[self.index],
            ),
            "benchmark gps": SimpleNamespace(
                enable=Mock(), getValues=lambda: (-4.65 + self.time * 0.2, -4.2, 0.0449),
                getSpeed=lambda: 0.2,
            ),
            "bumper_left": SimpleNamespace(enable=Mock(), getValue=lambda: 0.0),
            "bumper_right": SimpleNamespace(enable=Mock(), getValue=lambda: 0.0),
        }

    def step(self, timestep: int) -> int:
        if timestep == 0:
            self.flushes += 1
            return 0
        if self.index + 1 == len(self.distances):
            return -1
        self.index += 1
        self.time = (self.index + 1) * timestep / 1000
        return 0

    def getTime(self) -> float:
        return self.time

    def getBasicTimeStep(self) -> float:
        return 16.0

    def getWorldPath(self) -> str:
        return str(ROOT / "worlds/apartment.wbt")

    def getName(self) -> str:
        return "Create"

    def getModel(self) -> str:
        return "IRobot Create"

    def getDevice(self, name: str):
        return self.devices.get(name)


class BaselineChecks(unittest.TestCase):
    def recorded_run(self, robot: FakeRobot, directory: str, brain=None, **config):
        with redirect_stdout(io.StringIO()):
            path = run(robot, Config(results_dir=directory, **config), brain=brain)
        records = [json.loads(line) for line in (path / "steps.jsonl").read_text().splitlines()]
        return path, records

    def test_results_use_datetime_and_preserve_same_second_runs(self):
        fixed = datetime(2026, 10, 4, 12, 34, 56, tzinfo=timezone.utc)
        expected = fixed.astimezone().strftime("%Y-%m-%d_%H-%M-%S")
        with tempfile.TemporaryDirectory() as directory, patch("experiment.logger.datetime") as clock:
            clock.now.return_value = fixed
            first = RunLogger(Path(directory), Config())
            first.write("test", value=1)
            first.close()
            second = RunLogger(Path(directory), Config())
            second.close()
            self.assertEqual(first.directory.name, expected)
            self.assertEqual(second.directory.name, expected + "_1")
            self.assertNotEqual(first.run_id, second.run_id)
            self.assertEqual(json.loads((first.directory / "steps.jsonl").read_text())["value"], 1)

    def test_history_is_bounded_and_wait_duration_tracks_applied_action(self):
        seen = []
        def decide(observation):
            seen.append(observation)
            return Decision(Action.WAIT if len(seen) <= 3 else Action.CONTINUE, "history")
        brain = SimpleNamespace(name="history", version="test", decide=decide)
        with tempfile.TemporaryDirectory() as directory:
            self.recorded_run(FakeRobot([1.0] * 60), directory, brain=brain)
        self.assertEqual(seen[0].decision_history, ())
        self.assertEqual(len(seen[-1].decision_history), 10)
        self.assertEqual(len(seen[3].decision_history), 3)
        self.assertGreater(seen[3].waiting_duration_s, 0)
        self.assertEqual(seen[4].waiting_duration_s, 0)
        self.assertEqual(seen[-1].decision_history[-1]["requested"], "CONTINUE")
        self.assertTrue(all(item["accepted"] for item in seen[-1].decision_history))
        restored = Observation.from_dict(json.loads(json.dumps(seen[-1].to_dict())))
        self.assertEqual(restored, seen[-1])
        self.assertEqual(len(seen[3].decision_history), 3)  # Old snapshots stay unchanged.

    def test_typed_offline_observation_and_baseline(self):
        for distance, expected in ((0.449, Action.CONTINUE), (0.45, Action.CONTINUE), (None, Action.WAIT)):
            observation = Observation(0.2, distance, None, None, Action.CONTINUE)
            restored = Observation.from_dict(json.loads(json.dumps(observation.to_dict())))
            self.assertEqual(restored, observation)
            self.assertEqual(RuleBasedBrain(0.45).decide(restored).action, expected)
        with self.assertRaises(ValueError):
            Observation.from_dict({**observation.to_dict(), "current_action": "MOVE_FORWARD"})
        with self.assertRaises(ValueError):
            Decision("CONTINUE", "bad")
        with self.assertRaises(ValueError):
            Decision(Action.WAIT, "bad", confidence=float("nan"))

    def test_contact_recovery_without_periodic_turns(self):
        clear = Observation(0.2, 1.0, False, False, Action.CONTINUE, contact_detected=False)
        straight = RuleBasedBrain(0.45)
        near = replace(clear, front_distance=0.01)
        self.assertEqual([straight.decide(near).action for _ in range(1000)],
                         [Action.CONTINUE] * 1000)
        contact = replace(clear, contact_detected=True)
        brain = RuleBasedBrain(0.45)
        actions = [brain.decide(contact).action]
        actions.extend(brain.decide(clear).action for _ in range(150))
        self.assertEqual(actions[:8], [Action.BACK_UP] * 8)
        self.assertIn(actions[8], (Action.TURN_LEFT, Action.TURN_RIGHT))
        self.assertIn(Action.CONTINUE, actions[9:])
        repeated = RuleBasedBrain(0.45)
        self.assertEqual(actions, [repeated.decide(contact).action] +
                         [repeated.decide(clear).action for _ in range(150)])
        other = RuleBasedBrain(0.45, seed=1)
        self.assertNotEqual(actions, [other.decide(contact).action] +
                            [other.decide(clear).action for _ in range(150)])
        self.assertEqual(brain.decide(replace(clear, front_distance=None)).action, Action.WAIT)

        safety = SafetyLayer(0.12, 0.16)
        state = SensorState(0, 0.05, 0, None, True, False)
        self.assertEqual(safety.apply(Action.CONTINUE, state).action, Action.WAIT)
        self.assertEqual(safety.apply(Action.BACK_UP, state).action, Action.BACK_UP)
        self.assertEqual(safety.apply(Action.TURN_LEFT, state).action, Action.WAIT)
        self.assertEqual(safety.apply(Action.TURN_LEFT, replace(state, bumper_left=False)).action,
                         Action.TURN_LEFT)
        self.assertEqual(safety.apply(Action.BACK_UP, replace(state, bumper_right=None)).action,
                         Action.WAIT)
        with tempfile.TemporaryDirectory() as directory:
            robot = FakeRobot([1.0] * 250)
            robot.devices["bumper_left"].getValue = lambda: float(robot.index < 8)
            _, records = self.recorded_run(robot, directory)
            steps = [r for r in records if r["record_type"] == "step"]
            self.assertTrue(steps[0]["model_input"]["contact_detected"])
            self.assertEqual(steps[0]["applied_action"], "BACK_UP")
            self.assertIn(steps[32]["applied_action"], ("TURN_LEFT", "TURN_RIGHT"))
            self.assertTrue(any(r["applied_action"] == "CONTINUE" for r in steps[33:]))

    def test_hard_stop_is_evaluated_between_decisions(self):
        # Stop on contact between decisions, but allow approach to the obstacle.
        with tempfile.TemporaryDirectory() as directory:
            robot = FakeRobot([1.0, 0.05, 0.13, 1.0, 1.0])
            robot.devices["bumper_left"].getValue = lambda: float(robot.index == 2)
            path, records = self.recorded_run(robot, directory)
            steps = [record for record in records if record["record_type"] == "step"]
            self.assertEqual([step["applied_action"] for step in steps],
                             ["CONTINUE", "CONTINUE", "WAIT", "CONTINUE", "CONTINUE"])
            self.assertEqual(steps[1]["requested_action"], "CONTINUE")
            self.assertTrue(steps[2]["safety_override"])
            self.assertEqual(steps[2]["safety_reason"], "bumper_contact")
            self.assertIsNone(steps[1]["decision"])
            self.assertIsNone(steps[1]["model_input"])
            self.assertEqual(steps[0]["model_input"], steps[0]["sensor_observation"])
            self.assertNotIn("position_m", steps[0]["model_input"])
            self.assertNotIn("ground_truth", steps[0]["model_input"])
            timing = steps[0]["timing"]
            self.assertLessEqual(timing["perception_end_wall_ms"], timing["decision_start_wall_ms"])
            self.assertLessEqual(timing["decision_end_wall_ms"], timing["actuation_command_wall_ms"])
            self.assertIsNone(timing["event_time_s"])
            self.assertEqual(records[-1]["status"], "simulator_ended")
            self.assertEqual(records[-1]["decisions"], 2)
            self.assertFalse(records[-1]["stop_command_flushed"])
            self.assertEqual(robot.flushes, 0)
            self.assertIsNone(records[-1]["collision"])
            robot.devices["left wheel motor"].setVelocity.assert_called_with(0.0)
            for relative, digest in records[0]["source_sha256"].items():
                data = (path / "snapshot" / relative).read_bytes()
                self.assertEqual(hashlib.sha256(data).hexdigest(), digest)
            repeated_robot = FakeRobot(robot.distances)
            repeated_robot.devices["bumper_left"].getValue = lambda: float(repeated_robot.index == 2)
            _, repeated = self.recorded_run(repeated_robot, directory)
            self.assertNotEqual(records[0]["run_id"], repeated[0]["run_id"])
            self.assertEqual([r["applied_action"] for r in repeated if r["record_type"] == "step"],
                             [r["applied_action"] for r in steps])

    def test_invalid_sensors_contact_and_release(self):
        safety = SafetyLayer(0.12, 0.16)
        state = SensorState(0.016, None, 0.2, None, False, False)
        self.assertEqual(safety.apply(Action.CONTINUE, state).reason, "invalid_safety_sensor")
        self.assertEqual(safety.apply(Action.BACK_UP, replace(state, front_distance_m=0.13)).action, Action.BACK_UP)
        self.assertEqual(safety.apply(Action.CONTINUE, replace(state, front_distance_m=0.16)).action, Action.CONTINUE)
        self.assertEqual(safety.apply(Action.CONTINUE, replace(state, front_distance_m=1, bumper_left=True)).reason,
                         "bumper_contact")
        self.assertEqual(safety.apply(Action.CONTINUE, replace(state, front_distance_m=1, bumper_right=None)).reason,
                         "invalid_safety_sensor")
        with tempfile.TemporaryDirectory() as directory:
            _, records = self.recorded_run(FakeRobot([float("nan")]), directory)
            self.assertIsNone(records[1]["sensors"]["front_distance_m"])
            self.assertEqual(records[1]["applied_action"], "WAIT")

    def test_adapter_error_and_unserializable_output_fall_back_to_stop(self):
        for bad_decide in (
            Mock(side_effect=RuntimeError("adapter unavailable")),
            Mock(return_value={"action": "CONTINUE"}),
            Mock(return_value=Decision(Action.CONTINUE, "bad", raw_output=object())),
        ):
            with tempfile.TemporaryDirectory() as directory:
                brain = SimpleNamespace(name="bad", version="test", decide=bad_decide)
                _, records = self.recorded_run(FakeRobot([1.0]), directory, brain)
                self.assertEqual(records[1]["applied_action"], "WAIT")
                self.assertTrue(records[1]["decision"]["metadata"]["fallback"])
                self.assertEqual(records[-1]["fallback_count"], 1)

    def test_controller_error_stops_and_records_failed_run(self):
        with tempfile.TemporaryDirectory() as directory:
            robot = FakeRobot([1.0])
            robot.devices["front distance"].getValue = Mock(side_effect=RuntimeError("sensor failed"))
            with redirect_stdout(io.StringIO()), self.assertRaisesRegex(RuntimeError, "sensor failed"):
                run(robot, Config(results_dir=directory))
            robot.devices["left wheel motor"].setVelocity.assert_called_with(0.0)
            log = next(Path(directory).glob("*/steps.jsonl"))
            records = [json.loads(line) for line in log.read_text().splitlines()]
            self.assertEqual(records[-1]["status"], "error")
            self.assertEqual(records[-1]["steps"], 0)
            self.assertTrue(records[-1]["stop_command_flushed"])

    def test_world_and_configuration_mismatches_are_rejected(self):
        robot = FakeRobot([1.0])
        for config in (Config(seed=1), Config(timestep_ms=32, decision_interval_ms=64)):
            with self.assertRaises(ValueError):
                validate_world(robot, config, ROOT)
        robot.time = 1
        with self.assertRaises(ValueError):
            validate_world(robot, Config(), ROOT)
        for changes in ({"decision_interval_ms": 17}, {"forward_speed_m_s": -1},
                        {"critical_distance_m": 0.5}, {"seed": True}):
            with self.assertRaises(ValueError):
                Config(**changes)

    def test_duration_reached_and_all_discrete_motor_commands(self):
        with tempfile.TemporaryDirectory() as directory:
            robot = FakeRobot([1.0, 1.0, 1.0])
            _, records = self.recorded_run(robot, directory, max_simulation_time_s=0.032)
            self.assertEqual(records[-1]["status"], "duration_reached")
            self.assertEqual(records[-1]["steps"], 2)
            self.assertEqual(robot.time, 0.032)
            self.assertTrue(records[-1]["stop_command_flushed"])
            self.assertEqual(robot.flushes, 1)
        from controllers.cleaner.actuators import Actuators
        actuators = Actuators(robot, Config())
        self.assertEqual(set(actuators.commands), set(Action))
        self.assertLess(actuators.apply(Action.TURN_LEFT)[0], 0)
        self.assertGreater(actuators.apply(Action.TURN_LEFT)[1], 0)
        self.assertEqual(actuators.apply(Action.WAIT), (0, 0))


if __name__ == "__main__":
    unittest.main()
