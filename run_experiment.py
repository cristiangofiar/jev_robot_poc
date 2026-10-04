"""Repeat runs in fresh Webots processes/projects with the same controlled seed sequence."""

import argparse
import json
import os
import re
import shutil
import subprocess
from dataclasses import asdict, replace
from pathlib import Path
from uuid import uuid4

from experiment.config import Config
from experiment.environment import load_env
from experiment.scenario import FallingBoxScenario

ROOT = Path(__file__).resolve().parent


def prepare_batch(root: Path, config: Config, runs: int) -> Path:
    if runs < 1:
        raise ValueError("runs must be positive")
    FallingBoxScenario.load(root, config)  # Validate the selected scenario before launch.
    batch = root / "results/batches" / uuid4().hex
    batch.mkdir(parents=True)
    plan = []
    for index in range(runs):
        project = batch / "projects" / f"run_{index:03d}"
        project.mkdir(parents=True)
        for directory in ("controllers", "experiment"):
            shutil.copytree(root / directory, project / directory,
                            ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        for script in root.glob("*.py"):
            shutil.copy2(script, project / script.name)
        manifest = root / ".local_models/manifest.json"
        if manifest.is_file():
            (project / ".local_models").mkdir()
            shutil.copy2(manifest, project / ".local_models/manifest.json")
        effective = replace(config, seed=config.seed + index, batch_mode=True,
                            experiment_id=batch.name, results_dir=str(batch / "raw"))
        config_path = project / "experiment/configs/batch.json"
        config_path.write_text(json.dumps(asdict(effective), indent=2) + "\n")
        world_text = (root / "worlds/apartment.wbt").read_text()
        world_text, seeds = re.subn(r"(?m)^(\s*randomSeed\s+)\d+\s*$", rf"\g<1>{effective.seed}", world_text)
        world_text, controllers = re.subn(
            r'controllerArgs\s*\[\s*"--config"\s*"[^"]+"\s*\]',
            'controllerArgs [ "--config" "experiment/configs/batch.json" ]', world_text)
        if seeds != 1 or controllers != 2:
            raise ValueError("Expected one WorldInfo seed and two experiment controllerArgs")
        (project / "worlds").mkdir()
        world = project / "worlds/apartment.wbt"
        world.write_text(world_text)
        plan.append({"index": index, "seed": effective.seed, "brain": effective.brain,
                     "world": str(world), "config": asdict(effective)})
    (batch / "plan.json").write_text(json.dumps({"simulation_mode": "realtime", "runs": plan}, indent=2) + "\n")
    return batch


def execute_batch(batch: Path, executable: str, timeout_s: float) -> list[dict]:
    if not Path(executable).is_file():
        raise FileNotFoundError("Set WEBOTS_EXECUTABLE in .env to your installed Webots executable")
    version = subprocess.run([executable, "--version"], capture_output=True, text=True, timeout=30, check=True)
    version_text = version.stdout.strip().removeprefix("Webots version: ")
    child_env = {**os.environ, "WEBOTS_VERSION": version_text}
    (batch / "simulator.json").write_text(json.dumps({"executable": executable, "version": version_text}, indent=2) + "\n")
    statuses = []
    for run in json.loads((batch / "plan.json").read_text())["runs"]:
        with (batch / f"webots_{run['index']:03d}.log").open("w") as log:
            try:
                process = subprocess.run([executable, "--batch", "--mode=realtime", "--no-rendering",
                                          "--stdout", "--stderr", run["world"]],
                                         stdout=log, stderr=subprocess.STDOUT, timeout=timeout_s, env=child_env)
                status = {"index": run["index"], "exit_code": process.returncode, "timed_out": False}
            except subprocess.TimeoutExpired:
                status = {"index": run["index"], "exit_code": None, "timed_out": True}
        statuses.append(status)
        (batch / "execution.json").write_text(json.dumps(statuses, indent=2) + "\n")
        print(f"run {run['index']}: {status}", flush=True)
    return statuses


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "experiment/configs/falling_object_close.json")
    parser.add_argument("--brain", choices=("rules", "threshold", "kev", "laya", "jev"), default="rules")
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--scenario", choices=("apartment_static", "falling_object_close"))
    parser.add_argument("--webots")
    parser.add_argument("--timeout", type=float, default=120)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    load_env(ROOT / ".env")
    config = replace(Config.load(args.config), brain=args.brain, seed=args.seed)
    if args.scenario:
        config = replace(config, scenario_id=args.scenario)
    if not args.dry_run:
        from controllers.cleaner.brains import create_brain
        brain = create_brain(config)  # Missing Jev credentials fail before simulator startup.
    batch = prepare_batch(ROOT, config, args.runs)
    print(f"Prepared {batch}", flush=True)
    if not args.dry_run:
        from controllers.cleaner.brains.base import Action, Observation
        # Keep cold Metal compilation/connection setup outside the scored window.
        warmup = brain.decide(Observation(0.0, 2.0, False, False, Action.WAIT, contact_detected=False))
        (batch / "warmup.json").write_text(json.dumps(asdict(warmup), indent=2, allow_nan=False) + "\n")
        statuses = execute_batch(batch, args.webots or os.environ.get("WEBOTS_EXECUTABLE", ""), args.timeout)
        from experiment.metrics import analyze
        analyze(batch)
        if any(item["exit_code"] != 0 for item in statuses):
            raise SystemExit("Some repetitions failed; inspect execution.json and Webots logs")


if __name__ == "__main__":
    main()
