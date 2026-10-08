"""Probe the movement contract with synthetic observations, without Webots."""
import argparse
import json
from pathlib import Path
from mission.brain import Brain
from mission.environment import load_env


def observation():
    return {"simulation_s": 1, "revision": 1, "pose": [0, 0, 0], "battery_pct": 90,
            "time_remaining_s": 600, "explore_remaining_s": 240, "return_due": False, "return_reason": None,
            "goal": {"kind": "sample", "id": "S1", "distance_m": 1.3, "bearing_deg": -30},
            "base": {"distance_m": 0, "bearing_deg": 0},
            "candidates": [{"id": "S1", "distance_m": 1.3, "bearing_deg": -30, "visible": True, "inspected": False,
                            "last_seen_ago_s": 0, "inspect_ready": False, "collect_ready": False}],
            "collected": [], "delivered": [], "sensors_valid": True, "lidar_m": {"front": 1, "left": 4, "right": 4, "rear": 4},
            "motion": {"speed_m_s": 0, "yaw_deg": 0, "roll_deg": 0, "pitch_deg": 0},
            "exploration": {"visited_pct": 5, "unvisited_nearby_sectors": {"front": 20, "left": 10, "right": 10, "rear": 0},
                            "measure": "GPS track neighbourhoods; no discovery guarantee"},
            "progress": {"window_s": 8, "displacement_m": 0, "turn_deg": 0, "no_progress": True, "repeated_action": False},
            "history": [], "safety": None, "last_safety_intervention": None, "fallback": None, "action_duration_s": .8, "stopped_s": 2,
            "boundary_clearance_m": {"x_min": 1, "x_max": 3.6, "y_min": 2.1, "y_max": 2.1}, "frame": None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--brain', choices=('rules', 'laya', 'kev', 'jev'), required=True)
    args = parser.parse_args()
    load_env(Path(__file__).resolve().parent / '.env')
    brain = Brain(args.brain, timeout=30)
    for bearing in (-30, 30, 0):
        state = observation()
        state['goal']['bearing_deg'] = state['candidates'][0]['bearing_deg'] = bearing
        print(json.dumps(brain.decide(state), indent=2))


if __name__ == '__main__':
    main()
