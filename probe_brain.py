"""Probe the mission contract without starting Webots."""
import argparse
import json
from pathlib import Path
from mission.brain import Brain
from mission.environment import load_env


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--brain', choices=('rules', 'laya', 'kev', 'jev'), required=True)
    args = parser.parse_args()
    load_env(Path(__file__).resolve().parent / '.env')
    brain = Brain(args.brain, timeout=30)
    for inspected, collected in ((False, []), (True, []), (True, ['S1', 'S2', 'S3'])):
        observation = {'simulation_s': 1, 'revision': 1, 'battery_pct': 90, 'candidates': [] if collected else [{'id': 'S1', 'distance_m': .6, 'inspected': inspected}],
                       'collected': collected, 'delivered': [], 'sensors_valid': True, 'obstacle': False, 'history': []}
        print(json.dumps(brain.decide(observation), indent=2))


if __name__ == '__main__':
    main()
