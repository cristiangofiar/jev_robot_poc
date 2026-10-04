"""Event timing, run linkage and isolation checks using Webots API doubles."""

import io
import json
import re
import tempfile
import unittest
from contextlib import redirect_stdout
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

from controllers.cleaner.cleaner import ROOT, run as run_cleaner
from controllers.object_spawner.object_spawner import attach_run, run, shared_contact_points
from experiment.config import Config
from experiment.scenario import FallingBoxScenario
from test_baseline import FakeRobot


class FakeNode:
    def __init__(self, position=(0, 0, 0)):
        self.position = list(position)
        self.rotation = [0, 0, 1, 0]
        self.contacts = []
        self.removed = False

    def getField(self, name):
        if name == "translation":
            return SimpleNamespace(setSFVec3f=lambda value: setattr(self, "position", value))
        return SimpleNamespace(setSFRotation=lambda value: setattr(self, "rotation", value))

    def resetPhysics(self):
        pass

    def getPosition(self):
        return self.position

    def getOrientation(self):
        return [1, 0, 0, 0, 1, 0, 0, 0, 1]

    def getVelocity(self):
        return [0, 0, 0, 0, 0, 0]

    def getContactPoints(self, include_descendants):
        return self.contacts

    def remove(self):
        self.removed = True


class FakeSupervisor(FakeRobot):
    def __init__(self, messages, samples=127, actual_robot_position=None, pair_contact=False):
        super().__init__([1.0] * samples)
        self.robot = FakeNode()
        self.box = None
        self.imports = []
        self.messages = [json.dumps(message) for message in messages]
        self.actual_robot_position = actual_robot_position
        self.pair_contact = pair_contact
        self.devices["experiment lifecycle"] = SimpleNamespace(
            enable=Mock(), getQueueLength=lambda: len(self.messages),
            getString=lambda: self.messages[0], nextPacket=lambda: self.messages.pop(0),
        )

    def getFromDef(self, name):
        return self.robot if name == "CLEANER" else self.box

    def getRoot(self):
        return SimpleNamespace(getField=lambda name: SimpleNamespace(importMFNodeFromString=self.import_node))

    def import_node(self, index, source):
        self.imports.append((self.time, source))
        position = re.search(r"translation\s+([^\n]+)", source)[1]
        self.box = FakeNode(tuple(map(float, position.split())))
        # Prerecorded contact points, not a fake physics claim.
        self.box.contacts = [SimpleNamespace(point=[-6.15, -5.2, 0.0])]
        if self.pair_contact:
            self.robot.contacts = [SimpleNamespace(point=[-6.15, -5.2, 0.0])]

    def step(self, timestep):
        result = super().step(timestep)
        if timestep and result != -1 and self.actual_robot_position is not None:
            self.robot.position = list(self.actual_robot_position)
        return result


class ScenarioChecks(unittest.TestCase):
    def make_run(self, directory, **changes):
        config = Config(scenario_id="falling_object_close", results_dir=directory,
                        max_simulation_time_s=2.032, stop_before_contact=True, **changes)
        cleaner = FakeRobot([1.0] * 124 + [0.05] * 3)
        emitter = Mock()
        cleaner.devices["experiment lifecycle"] = emitter
        with redirect_stdout(io.StringIO()):
            path = run_cleaner(cleaner, config)
        message = json.loads(emitter.send.call_args.args[0])
        return config, path, message

    def test_event_is_fixed_world_position_not_actual_robot_position(self):
        with tempfile.TemporaryDirectory() as directory:
            config, first, message = self.make_run(directory, forward_speed_m_s=0.4)
            scenario = FallingBoxScenario.load(ROOT, config)
            supervisors = []
            for actual_position in ((-4.65, -5.2, 0.0449), (-4.0, -5.0, 0.0449)):
                if supervisors:
                    config, path, message = self.make_run(directory, forward_speed_m_s=0.4)
                else:
                    path = first
                supervisor = FakeSupervisor([message], actual_robot_position=actual_position)
                with redirect_stdout(io.StringIO()):
                    self.assertEqual(run(supervisor, config).resolve(), path.resolve())
                records = [json.loads(line) for line in (path / "ground_truth.jsonl").read_text().splitlines()]
                event = next(record for record in records if record["record_type"] == "event_released")
                self.assertEqual(event["release_command_time_s"], 2.0)
                self.assertEqual(event["release_position_m"], list(scenario.release_position(0.4)))
                self.assertEqual(event["actual_robot_position_m"], list(actual_position))
                self.assertEqual(records[-1]["outcome"], "no_object_contact_observed")
                self.assertTrue(records[-1]["object_removed"])
                self.assertTrue(supervisor.box.removed)
                self.assertEqual(len(supervisor.imports), 1)
                cleaner_records = [json.loads(line) for line in (path / "steps.jsonl").read_text().splitlines()]
                self.assertEqual(records[0]["run_id"], cleaner_records[0]["run_id"])
                for record in cleaner_records:
                    if record["record_type"] == "step" and record["model_input"] is not None:
                        self.assertNotIn("ground_truth", record["model_input"])
                        self.assertNotIn("scenario_id", record["model_input"])
                        self.assertNotIn("event_time_s", record["model_input"])
                self.assertEqual(cleaner_records[-2]["requested_action"], "CONTINUE")
                self.assertEqual(cleaner_records[-2]["applied_action"], "STOP")
                supervisors.append(supervisor)
            self.assertEqual(supervisors[0].imports, supervisors[1].imports)

    def test_object_contacts_are_distinguished_from_floor_contact(self):
        point = lambda xyz: SimpleNamespace(point=xyz)
        self.assertEqual(shared_contact_points([point([0, 0, 0])], [point([1, 0, 0])]), [])
        self.assertEqual(shared_contact_points([point([1, 0, 0])], [point([1, 0, 0])]), [[1, 0, 0]])
        with tempfile.TemporaryDirectory() as directory:
            config, path, message = self.make_run(directory)
            supervisor = FakeSupervisor([message], pair_contact=True)
            with redirect_stdout(io.StringIO()):
                run(supervisor, config)
            records = [json.loads(line) for line in (path / "ground_truth.jsonl").read_text().splitlines()]
            self.assertEqual(records[-1]["outcome"], "collision")
            self.assertTrue(records[-1]["collision_with_spawned_object"])

    def test_bad_link_or_duplicate_cannot_spawn_or_overwrite_results(self):
        with tempfile.TemporaryDirectory() as directory:
            config, path, message = self.make_run(directory)
            for bad in (
                {**message, "directory_name": "../other"},
                {**message, "parameters": {**message["parameters"], "seed": 1}},
                {**message, "context": {**message["context"], "run_id": "0" * 32}},
            ):
                with self.assertRaises(ValueError):
                    attach_run(ROOT, config, bad)
            supervisor = FakeSupervisor([message, message])
            with redirect_stdout(io.StringIO()), self.assertRaisesRegex(ValueError, "Duplicate"):
                run(supervisor, config)
            self.assertEqual(supervisor.imports, [])
            records = [json.loads(line) for line in (path / "ground_truth.jsonl").read_text().splitlines()]
            self.assertEqual(records[-1]["outcome"], "incomplete")
            with self.assertRaises(FileExistsError):
                attach_run(ROOT, config, message)

    def test_missing_metadata_aborts_before_release_and_static_is_untouched(self):
        config = Config(scenario_id="falling_object_close", max_simulation_time_s=2.032)
        supervisor = FakeSupervisor([])
        with self.assertRaisesRegex(TimeoutError, "metadata"):
            run(supervisor, config)
        self.assertEqual(supervisor.imports, [])
        static = FakeSupervisor([])
        self.assertIsNone(run(static, Config()))
        self.assertEqual(static.time, 0)
        self.assertEqual(static.robot.position, [0, 0, 0])

    def test_scenario_validation_and_nominal_counterfactual(self):
        config = Config(scenario_id="falling_object_close")
        scenario = FallingBoxScenario.load(ROOT, config)
        self.assertTrue(scenario.nominal_ground_truth(0.4)["nominal_collision_expected"])
        self.assertFalse(replace(scenario, lateral_offset_m=2).nominal_ground_truth(0.4)["nominal_collision_expected"])
        for changes in ({"box_mass_kg": -1}, {"box_size_m": (0, 1, 1)},
                        {"initial_translation_m": (float("nan"), 0, 0)}, {"release_time_s": True}):
            with self.assertRaises(ValueError):
                replace(scenario, **changes)
        for changes in ({"scenario_id": "../secret"}, {"max_simulation_time_s": 2.0}):
            with self.assertRaises(ValueError):
                FallingBoxScenario.load(ROOT, replace(config, **changes))


if __name__ == "__main__":
    unittest.main()
