"""HTTP boundaries, stale responses, reset plans and metric denominators."""

import io
import csv
import json
import os
import shutil
import tempfile
import unittest
from contextlib import redirect_stdout
from dataclasses import replace
from pathlib import Path
from threading import Event
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

from controllers.cleaner.brains import create_brain
from controllers.cleaner.brains.base import Action, Decision, Observation
from controllers.cleaner.brains.system_one import BrainError, NoRedirect, SystemOneBrain
from controllers.cleaner.cleaner import ROOT, run
from controllers.object_spawner.object_spawner import finish_batch, run as run_supervisor
from experiment.config import Config
from experiment.environment import load_env
from experiment.inference import PendingInference
from experiment.metrics import analyze, summarize
from run_experiment import prepare_batch
from test_baseline import FakeRobot
from test_scenario import FakeSupervisor
import test_scenario


def response():
    probabilities = {action.value: .02 for action in Action}
    probabilities["WAIT"] = .9
    return {"model": "jev-1.13.0", "answers": {"behavior": {
        "type": "choice", "choice": "WAIT", "confidence": .81, "probabilities": probabilities}},
        "usage": {"input_tokens": 100, "output_tokens": 0}}


class ModelChecks(unittest.TestCase):
    def test_action_space_excludes_stop(self):
        self.assertEqual(len(Action), 6)
        with self.assertRaises(ValueError):
            Action("STOP")
        with self.assertRaises(ValueError):
            Action("REPLAN")
        brain = SystemOneBrain("laya", "http://127.0.0.1:8009", "laya")
        self.assertNotIn("STOP", brain.questions["behavior"]["criteria"])
        self.assertNotIn("REPLAN", brain.questions["behavior"]["criteria"])

    def test_same_prompt_and_sensor_payload_for_all_adapters(self):
        observation = Observation(.4, .08, True, True, Action.CONTINUE, contact_detected=False)
        with patch.dict(os.environ, {"OPENROUTER_API_KEY": "test-secret"}, clear=True):
            adapters = [create_brain(Config(brain=name)) for name in ("kev", "laya", "jev")]
        payloads = [adapter.payload(observation) for adapter in adapters]
        self.assertEqual(payloads[0]["state"], payloads[1]["state"])
        self.assertEqual(payloads[1]["questions"], payloads[2]["questions"])
        self.assertEqual(set(payloads[0]["questions"]["behavior"]["criteria"]), {a.value for a in Action})
        self.assertNotIn("test-secret", json.dumps(payloads))
        self.assertNotIn("ground_truth", payloads[0]["state"])
        with patch.dict(os.environ, {}, clear=True), self.assertRaisesRegex(ValueError, "OPENROUTER_API_KEY"):
            create_brain(Config(brain="jev"))

    def test_wire_response_cost_validation_and_secret_handling(self):
        brain = SystemOneBrain("jev", "https://api.typesafe.ai", "jev-1.13.0", "test-secret", input_price=2, output_price=3)
        observation = Observation(.4, .08, True, True, Action.CONTINUE)
        fake = MagicMock()
        fake.open.return_value.__enter__.return_value.read.return_value = json.dumps(response()).encode()
        with patch("controllers.cleaner.brains.system_one.build_opener", return_value=fake):
            decision = brain.decide(observation)
        request = fake.open.call_args.args[0]
        self.assertEqual(request.get_header("Authorization"), "Bearer test-secret")
        self.assertEqual(request.full_url, "https://api.typesafe.ai/v1/systemone")
        self.assertEqual(decision.action, Action.WAIT)
        self.assertEqual(decision.confidence, .81)
        self.assertEqual(decision.probabilities[Action.WAIT], .9)
        self.assertAlmostEqual(decision.metadata["estimated_cost_usd"], .0002)
        self.assertNotIn("test-secret", json.dumps(decision.metadata))
        self.assertIsNone(NoRedirect().redirect_request(request, None, 302, "", {}, "https://elsewhere.invalid"))
        for change in ("invalid_choice", "missing_probability", "bad_sum", "bad_confidence", "bad_usage"):
            raw = response()
            answer = raw["answers"]["behavior"]
            if change == "invalid_choice": answer["choice"] = "MOVE"
            if change == "missing_probability": del answer["probabilities"]["TURN_RIGHT"]
            if change == "bad_sum": answer["probabilities"]["WAIT"] = .95
            if change == "bad_confidence": answer["confidence"] = True
            if change == "bad_usage": raw["usage"]["input_tokens"] = -1
            with self.assertRaises(ValueError): brain.decode(raw, 10)
        with self.assertRaises(ValueError): SystemOneBrain("jev", "http://remote.invalid", "jev")

    def test_jev_openrouter_wire_and_reported_cost(self):
        with patch.dict(os.environ, {"OPENROUTER_API_KEY": "test-secret"}, clear=True):
            brain = create_brain(Config(brain="jev"))
        raw = response()
        raw["model"] = "typesafe/jev-1.13-20260917"
        raw["usage"]["cost"] = .0000042
        fake = MagicMock()
        fake.open.return_value.__enter__.return_value.read.return_value = json.dumps(raw).encode()
        observation = Observation(.4, .08, True, True, Action.CONTINUE)
        with patch("controllers.cleaner.brains.system_one.build_opener", return_value=fake):
            decision = brain.decide(observation)
        request = fake.open.call_args.args[0]
        self.assertEqual(request.full_url, "https://openrouter.ai/api/v1/systemone")
        self.assertEqual(request.get_header("Authorization"), "Bearer test-secret")
        self.assertEqual(json.loads(request.data)["model"], "typesafe/jev-1.13")
        self.assertEqual(decision.metadata["served_model"], raw["model"])
        self.assertEqual(decision.metadata["estimated_cost_usd"], .0000042)
        self.assertEqual(decision.metadata["cost_source"], "provider")
        self.assertNotIn("test-secret", json.dumps(decision.metadata))
        for cost in (-1, True, "0.01", float("nan")):
            raw["usage"]["cost"] = cost
            with self.assertRaises(ValueError):
                brain.decode(raw, 1)

    def test_sensor_text_describes_history_and_unknowns(self):
        brain = SystemOneBrain("laya", "http://127.0.0.1:8009", "laya")
        state = brain.payload(Observation(0, 2, False, False, Action.WAIT, contact_detected=False))["state"]
        self.assertIn("Forward path is clear", state)
        self.assertIn("Previous behavior: waiting", state)
        self.assertNotIn("WAIT", state)
        unknown = brain.payload(Observation(None, None, None, None, Action.WAIT))["state"]
        self.assertIn("Forward path is unknown", unknown)
        self.assertIn("bumper state is unknown", unknown)

    def test_prompt_contains_history_and_wait_duration(self):
        brain = SystemOneBrain("kev", "http://127.0.0.1:8008", "kev-4b")
        observation = Observation(0, 2, False, False, Action.WAIT,
            simulation_time_s=4, waiting_duration_s=3,
            decision_history=({"time_s": 1.0, "requested": "CONTINUE", "applied": "WAIT", "accepted": True},))
        state = brain.payload(observation)["state"]
        self.assertIn("Continuously stationary for 3.000s", state)
        self.assertIn("1.000s: CONTINUE/WAIT", state)

    def test_env_preserves_exports_and_does_not_evaluate_shell(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {"EXPORTED": "keep"}, clear=True):
            path = Path(directory) / ".env"
            path.write_text('# comment\nEXPORTED=replace\nSECRET="$(echo untouched)"\n')
            load_env(path)
            self.assertEqual(os.environ["EXPORTED"], "keep")
            self.assertEqual(os.environ["SECRET"], "$(echo untouched)")
            path.write_text("bad-key=private-value")
            with self.assertRaisesRegex(ValueError, "line 1") as context: load_env(path)
            self.assertNotIn("private-value", str(context.exception))

    def test_async_timeout_and_stale_results_never_apply(self):
        release = Event()
        brain = SimpleNamespace(name="slow", decide=lambda obs: (release.wait(2), Decision(Action.CONTINUE, "slow"))[1])
        observation = Observation(.4, 1, True, False, Action.WAIT)
        pending = PendingInference(brain, observation, 0, 1)
        try:
            self.assertIsNone(pending.poll(.1, 3, 2))
            pending.started -= 4
            decision, reason = pending.poll(.2, 3, 2)
            self.assertEqual(reason, "deadline_exceeded")
            self.assertEqual(decision.action, Action.WAIT)
            self.assertIsNone(pending.poll(.3, 3, 2))
        finally:
            release.set()
            pending.thread.join(1)
        immediate = SimpleNamespace(name="fast", decide=lambda obs: Decision(Action.CONTINUE, "fast"))
        pending = PendingInference(immediate, observation, 0, 2)
        pending.thread.join(1)
        decision, reason = pending.poll(2.1, 3, 2)
        self.assertEqual(reason, "stale_observation")
        self.assertEqual(decision.action, Action.WAIT)

    def test_slow_brain_does_not_block_physics_and_stops_while_pending(self):
        release = Event()
        brain = SimpleNamespace(name="slow", version="test",
            decide=lambda obs: (release.wait(2), Decision(Action.CONTINUE, "slow"))[1])
        try:
            with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
                path = run(FakeRobot([1] * 4), Config(brain="laya", results_dir=directory, max_simulation_time_s=.064), brain=brain)
                records = [json.loads(line) for line in (path / "steps.jsonl").read_text().splitlines()]
                steps = [r for r in records if r["record_type"] == "step"]
                self.assertEqual(len(steps), 4)
                self.assertTrue(all(r["applied_action"] == "WAIT" for r in steps))
                self.assertEqual(records[-1]["status"], "duration_reached")
                self.assertEqual(records[-1]["decisions"], 1)
        finally:
            release.set()

    def test_contact_overrides_a_held_async_model_decision(self):
        inferences = []
        def start_inference(*args):
            pending = PendingInference(*args)
            inferences.append(pending)
            return pending
        class Robot(FakeRobot):
            def step(self, timestep):
                if timestep and inferences:
                    inferences[-1].thread.join(1)
                return super().step(timestep)
        robot = Robot([1] * 4)
        robot.devices["bumper_left"].getValue = lambda: float(robot.index == 2)
        brain = SimpleNamespace(name="immediate", version="test", decide=lambda obs: Decision(Action.CONTINUE, "immediate"))
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()), patch(
                "controllers.cleaner.cleaner.PendingInference", side_effect=start_inference):
            path = run(robot, Config(brain="laya", results_dir=directory, max_simulation_time_s=.064), brain=brain)
            records = [json.loads(line) for line in (path / "steps.jsonl").read_text().splitlines()]
            steps = [r for r in records if r["record_type"] == "step"]
            self.assertFalse(inferences[-1].thread.is_alive())
            self.assertEqual([r["applied_action"] for r in steps], ["WAIT", "CONTINUE", "WAIT", "CONTINUE"])
            self.assertTrue(steps[2]["safety_override"])
            self.assertEqual(steps[2]["safety_reason"], "bumper_contact")
            result = next(r for r in records if r["record_type"] == "decision_result")
            self.assertEqual(result["model_input"], steps[0]["sensor_observation"])


class BatchChecks(unittest.TestCase):
    def test_each_repetition_gets_its_own_world_and_seed_without_secrets(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ("controllers", "experiment", "worlds"):
                shutil.copytree(ROOT / name, root / name, ignore=shutil.ignore_patterns("__pycache__"))
            (root / ".env").write_text("OPENROUTER_API_KEY=private-test-secret\n")
            batch = prepare_batch(root, Config(scenario_id="falling_object_close", seed=7, brain="kev"), 2)
            plan = json.loads((batch / "plan.json").read_text())
            self.assertEqual([r["seed"] for r in plan["runs"]], [7, 8])
            self.assertEqual(plan["simulation_mode"], "realtime")
            for item in plan["runs"]:
                world = Path(item["world"])
                self.assertIn(f"randomSeed {item['seed']}", world.read_text())
                self.assertEqual(world.read_text().count('"experiment/configs/batch.json"'), 2)
                self.assertTrue(item["config"]["batch_mode"])
                self.assertFalse((world.parents[1] / ".env").exists())
            self.assertIn("randomSeed 0", (root / "worlds/apartment.wbt").read_text())

    def test_batch_waits_for_cleaner_end_before_quitting(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "steps.jsonl"
            path.write_text(json.dumps({"record_type": "step", "run_id": "test"}) + "\n")
            logger = SimpleNamespace(directory=Path(directory), run_id="test")
            supervisor = Mock()
            def finish(_):
                with path.open("a") as file:
                    file.write(json.dumps({"record_type": "run_end", "run_id": "test",
                        "status": "duration_reached", "stop_command_flushed": True}) + "\n")
                return 0
            supervisor.step.side_effect = finish
            finish_batch(supervisor, Config(), logger)
            supervisor.step.assert_called_once_with(16)
            supervisor.simulationQuit.assert_called_once_with(0)

    def test_incomplete_ground_truth_is_not_counted_as_collision_avoidance(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            fixture = test_scenario.ScenarioChecks()
            config, first, message = fixture.make_run(directory)
            run_supervisor(FakeSupervisor([message], pair_contact=True), config)
            self.assertTrue(summarize(first)["valid"])
            self.assertTrue(summarize(first)["collision_with_spawned_object"])
            config, second, message = fixture.make_run(directory)
            run_supervisor(FakeSupervisor([message]), config)
            gt = second / "ground_truth.jsonl"
            gt.write_text("\n".join(gt.read_text().splitlines()[:-1]) + "\n")
            rows = analyze(Path(directory))
            self.assertEqual(sum(row["valid"] for row in rows), 1)
            self.assertIsNone(next(row for row in rows if not row["valid"])["collision_with_spawned_object"])
            with (Path(directory) / "aggregate.csv").open() as file:
                aggregate = list(csv.DictReader(file))
            self.assertEqual(aggregate[0]["valid_runs"], "1")
            self.assertEqual(float(aggregate[0]["collision_rate"]), 1)
            self.assertIsNone(rows[0]["action_accuracy"])


if __name__ == "__main__":
    unittest.main()
