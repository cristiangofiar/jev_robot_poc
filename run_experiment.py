"""Launch a fresh Mars mission in installed Webots (macOS/Windows/Linux)."""
import argparse
import json
import math
import statistics
import os
import shutil
import subprocess
from datetime import datetime
from pathlib import Path
from threading import Thread
from time import perf_counter
from uuid import uuid4
from mission.brain import Brain
from mission.environment import load_env
from mission.logging import OUTPUT_SEPARATOR, brain_output

ROOT = Path(__file__).resolve().parent


def stream_line(output, line):
    output.write(line)
    output.flush()
    print(line, end='', flush=True)


def run_webots(command, env, output, timeout):
    """Show response blocks; keep Webots diagnostics in a separate file."""
    with subprocess.Popen(command, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, errors='replace', bufsize=1) as process:
        def forward():
            in_response = False
            with Path(output.name).with_name('webots-runtime.log').open('w', encoding='utf-8') as diagnostics:
                for line in process.stdout:
                    diagnostics.write(line)
                    diagnostics.flush()
                    separator = line.rstrip('\r\n') == OUTPUT_SEPARATOR
                    if separator or in_response:
                        stream_line(output, ('\n' if separator and not in_response else '') + line)
                    if separator:
                        in_response = not in_response
        reader = Thread(target=forward, daemon=True)
        reader.start()
        try:
            return process.wait(timeout=timeout)
        except BaseException:
            process.kill()
            process.wait()
            raise
        finally:
            reader.join(timeout=2)


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
              'latency_median_ms': round(statistics.median(r['latency_ms'] for r in responses if r['rejection'] is None), 1) if any(r['rejection'] is None for r in responses) else None,
              'accepted_decisions': sum(r['rejection'] is None and r['decision'].get('choice') is not None for r in responses),
              'fallbacks': [r['fallback'] for r in records if r['type'] == 'action' and r['fallback']],
              'safety_interventions': sum(r['type'] == 'safety' and r['reason'] is not None for r in records),
              'obstacle_contact_steps': sum(r['type'] == 'trajectory' and bool(r['obstacle_contacts']) for r in truth),
              'latencies_ms': [round(r['latency_ms'], 1) for r in responses if r['rejection'] is None],
              'rejected_wait_ms': [{'reason': r['rejection'], 'wait_ms': round(r['latency_ms'], 1) if r['latency_ms'] is not None else None} for r in responses if r['rejection']],
              'late_latencies_ms': [round(r['latency_ms'], 1) for r in records if r['type'] == 'late_response'],
              'discoveries': sum(r['type'] == 'detected' for r in records),
              'collections': sum(r['type'] == 'operation' and r['event'] == 'collect' and r['ok'] for r in truth),
              'inspections': sum(r['type'] == 'operation' and r['event'] == 'inspect' and r['ok'] for r in truth),
              'repeated_actions': sum(a.get('accepted', a.get('applied')) == b.get('accepted', b.get('applied')) for a, b in zip(
                  [r for r in records if r['type'] == 'action'], [r for r in records if r['type'] == 'action'][1:])),
              'no_progress_steps': sum(r['type'] == 'step' and r.get('no_progress', False) for r in records),
              'action_sources': {source: sum(r['type'] == 'action' and r['source'] == source for r in records)
                                 for source in ('model', 'deterministic', 'fallback')},
              'ground_truth_evaluation': next((r for r in reversed(truth) if r['type'] == 'end'), None),
              'physical_outcome': ends[-1] if ends else None}
    source = None
    results = [r for r in records if r['type'] == 'action_result']
    result['blocked_actions'] = sum(r['result'] == 'blocked' for r in results)
    result['interrupted_actions'] = sum(r['result'] == 'interrupted' for r in results)
    for record in records:
        if record['type'] == 'action':
            source = record['source']
        if record['type'] == 'step':
            record.setdefault('action_source', source)
    steps = [r for r in records if r['type'] == 'step']
    result['travel_m_by_source'] = {source: round(sum(math.dist(a['position'][:2], b['position'][:2])
        for a, b in zip(steps, steps[1:]) if a.get('action_source') == source and a['applied']['mode'] != 'stop'), 3)
        for source in ('model', 'deterministic', 'fallback')}
    result['stalled_intervals'] = sum(b['simulation_s'] - a['simulation_s'] >= 6 and math.dist(a['position'][:2], b['position'][:2]) < .015
        and abs((b['yaw'] - a['yaw'] + math.pi) % (2*math.pi) - math.pi) < .05
        for a, b in zip(steps[::40], steps[40::40]))
    (directory / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--brain', choices=('rules', 'laya', 'kev', 'jev', 'slow'), default='rules')
    parser.add_argument('--webots')
    parser.add_argument('--world', type=Path, default=ROOT / 'worlds/mars.wbt')
    parser.add_argument('--skip-warmup', action='store_true', help='Start physics even if the local inference service is unavailable')
    parser.add_argument('--mode', choices=('realtime', 'fast', 'pause'))
    parser.add_argument('--test', choices=('mission', 'sensors', 'motion', 'preconditions'), default='mission')
    parser.add_argument('--duration', type=float, default=600)
    parser.add_argument('--timeout', type=float, default=1200)
    parser.add_argument('--deadline', type=float, default=2)
    parser.add_argument('--action-duration', type=float, default=.8)
    parser.add_argument('--explore-budget', type=float, default=240)
    parser.add_argument('--coverage-target', type=float, default=65)
    parser.add_argument('--delay', type=float, default=0, help='Injected inference delay for fallback verification')
    parser.add_argument('--no-rendering', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if not all(math.isfinite(value) for value in (args.duration, args.timeout, args.deadline, args.action_duration, args.explore_budget, args.coverage_target, args.delay)):
        parser.error('Limits must be finite')
    if min(args.duration, args.timeout, args.deadline, args.action_duration, args.explore_budget) <= 0 or args.delay < 0:
        parser.error('Time limits must be positive; delay must be nonnegative')
    if args.action_duration > 1 or not 0 < args.coverage_target <= 100:
        parser.error('Action duration must be <=1s and coverage target in (0,100]')
    load_env(ROOT / '.env')
    executable = args.webots or find_webots()
    mode = args.mode or ('fast' if args.brain == 'rules' else 'realtime')
    directory = ROOT / 'results' / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + args.brain + '_' + uuid4().hex[:6])
    directory.mkdir(parents=True)
    child_env = {**os.environ, 'MARS_RUN_DIR': str(directory), 'MARS_BRAIN': args.brain, 'MARS_TEST': args.test,
                 'MARS_DURATION_S': str(args.duration), 'MARS_BATCH': '1', 'MARS_DEADLINE_S': str(args.deadline),
                 'MARS_ACTION_S': str(args.action_duration),
                 'MARS_EXPLORE_S': str(args.explore_budget), 'MARS_COVERAGE_PCT': str(args.coverage_target), 'MARS_TEST_DELAY_S': str(args.delay)}
    command = [executable, '--batch', '--stdout', '--stderr', '--port=1235', '--mode=' + mode]
    if args.no_rendering:
        command.append('--no-rendering')
    world = args.world.resolve()
    if not world.is_file() or world.suffix.lower() != '.wbt':
        parser.error('Expected an existing .wbt world')
    child_env['MARS_WORLD'] = str(world)
    command.append(str(world))
    (directory / 'launch.json').write_text(json.dumps({'brain': args.brain, 'test': args.test, 'mode': mode, 'duration_s': args.duration,
                                                       'deadline_s': args.deadline, 'delay_s': args.delay,
                                                       'action_duration_s': args.action_duration, 'explore_budget_s': args.explore_budget, 'coverage_target_pct': args.coverage_target, 'world': str(world)}, indent=2) + '\n')
    print(f'Run: {directory}', flush=True)
    if args.dry_run:
        print(json.dumps(command))
        return
    with (directory / 'webots.log').open('w', encoding='utf-8') as output:
        if args.brain in ('laya', 'kev', 'jev') and not args.skip_warmup:
            from probe_brain import observation
            state = observation()
            state.update(candidates=[], goal={"kind": "exploration"})
            started = perf_counter()
            try:
                answer = Brain(args.brain, timeout=30).decide(state)
            except Exception as exc:
                (directory / 'warmup.json').write_text(json.dumps({'error_type': type(exc).__name__, 'latency_ms': (perf_counter() - started) * 1000}) + '\n')
                raise SystemExit('Brain warmup failed; see ' + str(directory / 'warmup.json')) from None
            (directory / 'warmup.json').write_text(json.dumps(answer, indent=2) + '\n')
            stream_line(output, brain_output(args.brain, answer['raw_response'], (perf_counter() - started) * 1000, answer.get('http_status'), 'warmup'))
        try:
            version = subprocess.run([executable, '--version'], capture_output=True, text=True, timeout=30, check=True)
            child_env['WEBOTS_VERSION'] = version.stdout.strip()
            returncode = run_webots(command, child_env, output, args.timeout)
        except subprocess.TimeoutExpired:
            raise SystemExit('Webots wall deadline exceeded; see ' + str(directory / 'webots-runtime.log')) from None
        except (OSError, subprocess.CalledProcessError) as exc:
            with (directory / 'webots-runtime.log').open('a', encoding='utf-8') as diagnostics:
                diagnostics.write(f'Webots launch failed: {type(exc).__name__}\n')
            raise SystemExit('Webots launch failed; see ' + str(directory / 'webots-runtime.log')) from None
        if returncode != 0 or not (directory / 'steps.jsonl').exists() or not (directory / 'ground_truth.jsonl').exists():
            raise SystemExit('Webots failed; see ' + str(directory / 'webots-runtime.log'))
        summary = summarize(directory)
        expected = {'mission': ('returned_and_delivered', 'returned_empty'), 'sensors': ('sensors_complete',), 'motion': ('motion_complete',), 'preconditions': ('preconditions_complete',)}[args.test]
        if not summary['valid'] or summary['outcome'] not in expected:
            raise SystemExit('Mission/check incomplete; inspect run logs')


if __name__ == '__main__':
    main()
