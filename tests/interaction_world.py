"""Create close-range interaction or frontal-obstacle fixtures with official rover physics."""
import argparse
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def create(front_obstacle=False):
    world = (ROOT / 'worlds/mars.wbt').read_text()
    if front_obstacle:
        world = world.replace('DEF ROCK0 Solid { translation 1.35 -0.12 0.29708',
                              'DEF ROCK0 Solid { translation 0.8 0 0.29708')
        path = ROOT / 'worlds/safety_feedback.wbt'
        path.write_text(world)
        print(path)
        return
    for sid in ('S2', 'S3'):
        start = world.index(f'DEF {sid} Solid {{')
        depth = 0
        for end in range(world.index('{', start), len(world)):
            depth += (world[end] == '{') - (world[end] == '}')
            if depth == 0:
                world = world[:start] + world[end+1:]
                break
    world = world.replace('S1', 'S17')
    world = re.sub(r'(DEF S17 Solid \{\s+translation) [^\n]+', r'\1 0.58 0 0.235', world)
    path = ROOT / 'worlds/interaction.wbt'
    path.write_text(world)
    print(path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--front-obstacle', action='store_true')
    create(parser.parse_args().front_obstacle)
