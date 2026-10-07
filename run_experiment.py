"""Launch a fresh Mars mission in installed Webots (macOS/Windows/Linux)."""
import argparse
import json
import os
import shutil
import subprocess
from datetime import datetime
from pathlib import Path
from uuid import uuid4
from mission.brain import Brain
from mission.environment import load_env

ROOT = Path(__file__).resolve().parent


def find_webots():
    configured = os.environ.get('WEBOTS_EXECUTABLE')
    if configured and Path(configured).is_file():
        return configured
    candidates = [Path('/Applications/Webots.app/Contents/MacOS/webots')]
    if os.environ.get('WEBOTS_HOME'):
        home = Path(os.environ['WEBOTS_HOME'])
        candidates += [home / 'msys64/mingw64/bin/webots.exe', home / 'webots.exe', home / 'Contents/MacOS/webots', home / 'webots']
    if os.environ.get('ProgramFiles'):
        candidates.append(Path(os.environ['ProgramFiles']) / 'Webots/msys64/mingw64/bin/webots.exe')
    for path in candidates:
        if path.is_file():
            return str(path)
    found = shutil.which('webots')
    if found:
        return found
    raise FileNotFoundError('Set WEBOTS_EXECUTABLE to your installed Webots executable')


def summarize(directory):
    records = [json.loads(line) for line in (directory / 'steps.jsonl').read_text().splitlines()]
    truth = [json.loads(line) for line in (directory / 'ground_truth.jsonl').read_text().splitlines()]
    ends = [r for r in records if r['type'] == 'end']
    responses = [r for r in records if r['type'] == 'response']
    result = {'outcome': ends[-1]['outcome'] if ends else 'incomplete', 'valid': bool(ends and any(r['type'] == 'end' for r in truth)),
              'accepted_decisions': sum(r['rejection'] is None and r['decision'].get('choice') is not None for r in responses),
              'fallbacks': [r['rejection'] for r in responses if r['rejection']],
              'safety_interventions': sum(r['type'] == 'safety' and r['reason'] is not None for r in records),
              'obstacle_contact_steps': sum(r['type'] == 'trajectory' and bool(r['obstacle_contacts']) for r in truth),
              'latencies_ms': [round(r['latency_ms'], 1) for r in responses if r['rejection'] is None],
              'rejected_wait_ms': [{'reason': r['rejection'], 'wait_ms': round(r['latency_ms'], 1)} for r in responses if r['rejection']],
              'late_latencies_ms': [round(r['latency_ms'], 1) for r in records if r['type'] == 'late_response'],
              'local_route_failures': sum(r['type'] == 'fallback' and r['reason'] == 'no_route' for r in records) + sum(r['type'] == 'action' and r['fallback'] == 'no_route' for r in records),
              'physical_outcome': ends[-1] if ends else None}
    (directory / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--brain', choices=('rules', 'laya', 'kev', 'jev', 'slow'), default='rules')
    parser.add_argument('--webots')
    parser.add_argument('--mode', choices=('realtime', 'fast', 'pause'))
    parser.add_argument('--test', choices=('mission', 'sensors', 'motion'), default='mission')
    parser.add_argument('--duration', type=float, default=900)
    parser.add_argument('--timeout', type=float, default=1200)
    parser.add_argument('--deadline', type=float, default=2)
    parser.add_argument('--max-age', type=float, default=3)
    parser.add_argument('--delay', type=float, default=0, help='Injected inference delay for fallback verification')
    parser.add_argument('--no-rendering', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if min(args.duration, args.timeout, args.deadline, args.max_age) <= 0 or args.delay < 0:
        parser.error('Time limits must be positive; delay must be nonnegative')
    load_env(ROOT / '.env')
    executable = args.webots or find_webots()
    mode = args.mode or ('fast' if args.brain == 'rules' else 'realtime')
    directory = ROOT / 'results' / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + args.brain + '_' + uuid4().hex[:6])
    directory.mkdir(parents=True)
    child_env = {**os.environ, 'MARS_RUN_DIR': str(directory), 'MARS_BRAIN': args.brain, 'MARS_TEST': args.test,
                 'MARS_DURATION_S': str(args.duration), 'MARS_BATCH': '1', 'MARS_DEADLINE_S': str(args.deadline),
                 'MARS_MAX_AGE_S': str(args.max_age), 'MARS_TEST_DELAY_S': str(args.delay)}
    command = [executable, '--batch', '--stdout', '--stderr', '--port=1235', '--mode=' + mode]
    if args.no_rendering:
        command.append('--no-rendering')
    command.append(str(ROOT / 'worlds/mars.wbt'))
    (directory / 'launch.json').write_text(json.dumps({'brain': args.brain, 'test': args.test, 'mode': mode, 'duration_s': args.duration,
                                                       'deadline_s': args.deadline, 'max_age_s': args.max_age, 'delay_s': args.delay}, indent=2) + '\n')
    print(f'Run: {directory}', flush=True)
    if args.dry_run:
        print(json.dumps(command))
        return
    if args.brain in ('laya', 'kev', 'jev'):
        observation = {'simulation_s': 0, 'revision': 0, 'battery_pct': 100, 'candidates': [], 'collected': [], 'delivered': [], 'sensors_valid': True, 'obstacle': False, 'history': []}
        answer = Brain(args.brain, timeout=30).decide(observation)
        (directory / 'warmup.json').write_text(json.dumps(answer, indent=2) + '\n')
    version = subprocess.run([executable, '--version'], capture_output=True, text=True, timeout=30, check=True)
    child_env['WEBOTS_VERSION'] = version.stdout.strip()
    with (directory / 'webots.log').open('w') as output:
        try:
            result = subprocess.run(command, env=child_env, stdout=output, stderr=subprocess.STDOUT, timeout=args.timeout)
        except subprocess.TimeoutExpired:
            raise SystemExit('Webots wall deadline exceeded; see ' + str(directory / 'webots.log'))
    if result.returncode != 0 or not (directory / 'steps.jsonl').exists() or not (directory / 'ground_truth.jsonl').exists():
        raise SystemExit('Webots failed; see ' + str(directory / 'webots.log'))
    summary = summarize(directory)
    print(json.dumps(summary, indent=2), flush=True)
    expected = {'mission': 'success', 'sensors': 'sensors_complete', 'motion': 'motion_complete'}[args.test]
    if not summary['valid'] or summary['outcome'] != expected:
        raise SystemExit('Mission/check incomplete; inspect run logs')


if __name__ == '__main__':
    main()
