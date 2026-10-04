"""Exercise the real adapter on identical offline sensor states; not a robot benchmark."""

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from controllers.cleaner.brains import create_brain
from controllers.cleaner.brains.base import Action, Observation
from experiment.config import Config
from experiment.environment import load_env

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--brain", choices=("kev", "laya", "jev", "threshold", "rules"), required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--timeout", type=float, default=30)
    args = parser.parse_args()
    load_env(ROOT / ".env")
    brain = create_brain(Config(brain=args.brain, decision_timeout_s=args.timeout))
    samples = [
        ("clear_from_stop", Observation(0.0, 2.0, False, False, Action.WAIT, contact_detected=False)),
        ("clear_from_continue", Observation(0.0, 2.0, False, False, Action.CONTINUE, contact_detected=False)),
        ("clear", Observation(0.4, 1.2, True, False, Action.CONTINUE, contact_detected=False)),
        ("front_hazard", Observation(0.4, 0.08, True, True, Action.CONTINUE, contact_detected=False)),
        ("contact", Observation(0.0, 0.02, True, True, Action.WAIT, contact_detected=True)),
    ]
    records = []
    for name, observation in samples:
        decision = brain.decide(observation)
        records.append({"sample": name, "observation": observation.to_dict(), "decision": asdict(decision)})
        print(f"{name}: {decision.action.value}, confidence={decision.confidence}, {decision.latency_ms:.1f} ms", flush=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(records, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
