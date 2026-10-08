import json
import math
import unittest
from unittest.mock import patch
from threading import Event
from pathlib import Path
from tempfile import TemporaryDirectory
import zipfile
import io
import os
import subprocess
import sys
from collections import deque
from contextlib import redirect_stdout
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock
import setup_models
import run_experiment
from run_experiment import run_webots
from mission.brain import Brain, Pending, criteria, rules
from mission.baseline import Navigator
from mission.logging import OUTPUT_SEPARATOR, ActionExecution, Log, execution_history, start_run_log
from mission.navigation import Coverage, lidar_summary, relative, safety
from probe_brain import observation



class Checks(unittest.TestCase):
    def test_prompt_explains_lost_view_and_eligible_operations_without_filtering_moves(self):
        with patch.dict(os.environ, {'OPENROUTER_API_KEY': 'test-placeholder'}):
            for brain in ('laya', 'kev', 'jev'):
                o = observation()
                sample = o['candidates'][0]
                sample.update(distance_m=.035, bearing_deg=97, visible=False, last_seen_ago_s=67.9)
                o['goal'] = {'kind': 'sample', **sample}
                payload = Brain(brain).payload(o)
                context = payload['state'].split('Sample locations and total are unknown.')[0]
                self.assertIn('Nearby sample is NOT visible', context)
                self.assertIn('Reposition to restore its view', context)
                self.assertNotIn('remain stopped for operations', context)
                self.assertEqual(payload['questions']['mission']['criteria'], criteria(o))
                self.assertTrue({'FORWARD', 'TURN_LEFT', 'TURN_RIGHT', 'STOP'} <= set(criteria(o)))
                self.assertNotIn('REVERSE', criteria(o))
                self.assertNotIn('INSPECT_S1', criteria(o))
                self.assertIn('use REVERSE', payload['questions']['mission']['instructions'])
                sample.update(distance_m=.57, bearing_deg=0, visible=True, inspect_ready=True)
                o['goal'] = {'kind': 'sample', **sample}
                context = Brain(brain).payload(o)['state'].split('Sample locations and total are unknown.')[0]
                self.assertIn('INSPECT_S1 is available', context)
                self.assertNotIn('Reposition', context)
                sample.update(inspected=True, inspect_ready=False, collect_ready=True, visible=False)
                o['goal'] = {'kind': 'sample', **sample}
                self.assertIn('Inspection confirmed; COLLECT_S1 is available', Brain(brain).payload(o)['state'])
                sample.update(inspected=False, collect_ready=False, bearing_deg=40, visible=True)
                o['goal'] = {'kind': 'sample', **sample}
                self.assertIn('Face it within 18 degrees', Brain(brain).payload(o)['state'])
                o.update(goal={'kind': 'base', 'distance_m': .2, 'bearing_deg': 0})
                self.assertIn('Base reached; remain stopped for delivery', Brain(brain).payload(o)['state'])

    def test_ui_restarts_with_inherited_run_directory_preserve_previous_logs(self):
        with TemporaryDirectory() as temporary:
            configured = Path(temporary) / 'original_launch'
            configured.mkdir()
            (configured / 'launch.json').write_text('{"brain":"laya"}')
            (configured / 'webots.log').write_text('warmup output\n')
            first = start_run_log(configured, 'laya')
            self.assertEqual(first.directory, configured)
            first.write('end', outcome='time_limit')
            first.file.close()
            (configured / 'frames').mkdir()
            (configured / 'frames/request_0001.png').write_bytes(b'original frame')
            truth = Log(configured, 'ground_truth.jsonl')
            truth.write('end', outcome='time_limit')
            truth.file.close()
            original = {p.relative_to(configured): p.read_bytes() for p in configured.rglob('*') if p.is_file()}
            directories = {configured}
            for _ in range(3):
                restarted = start_run_log(configured, 'laya')
                current = restarted.directory
                self.assertEqual(current.parent, configured.parent)
                self.assertNotIn(current, directories)
                self.assertIn('_laya_', current.name)
                directories.add(current)
                (current / 'frames').mkdir()
                restarted.write('start', brain='laya')
                restarted.file.close()
                supervisor = Log(current, 'ground_truth.jsonl')
                supervisor.file.close()
            self.assertEqual(original, {p.relative_to(configured): p.read_bytes() for p in configured.rglob('*') if p.is_file()})
            for artifact in ('steps.jsonl', 'frames', 'ground_truth.jsonl'):
                partial = Path(temporary) / artifact.replace('.', '_')
                partial.mkdir()
                if artifact == 'frames':
                    (partial / artifact).mkdir()
                else:
                    (partial / artifact).write_text('previous supervisor output')
                restarted = start_run_log(partial, 'rules')
                self.assertNotEqual(restarted.directory, partial)
                restarted.file.close()
                if artifact == 'steps.jsonl':
                    self.assertEqual((partial / artifact).read_text(), 'previous supervisor output')
                else:
                    self.assertFalse((partial / 'steps.jsonl').exists())

    def test_all_brains_show_only_pretty_warmup_and_response_json(self):
        for brain, served_model in (('laya', 'laya'), ('kev', 'kev-4b'), ('jev', 'test-jev')):
            with self.subTest(brain=brain), TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
                root = Path(directory)
                world = root / 'worlds/mars.wbt'
                world.parent.mkdir()
                world.write_text('# test world')
                runtime = root / '.local_models' / f'{brain}-runtime.log'
                runtime.parent.mkdir()
                runtime.write_text('previous experiment\n')
                state = observation()
                state.update(candidates=[], goal={'kind': 'exploration'})
                raw = {'model': served_model, 'answers': {'mission': {'type': 'choice', 'choice': 'STOP',
                       'confidence': .8, 'probabilities': {k: int(k == 'STOP') for k in criteria(state)}}}}
                response = MagicMock()
                response.__enter__.return_value.status = 200
                response.__enter__.return_value.read.return_value = json.dumps(raw).encode()
                opener = Mock()
                opener.open.return_value = response
                script = '''import json,os
from pathlib import Path
from mission.logging import Log
directory=Path(os.environ['MARS_RUN_DIR'])
log=Log(directory,brain=os.environ['MARS_BRAIN'])
log.write('response',request_id=1,decision=json.loads(os.environ['TEST_DECISION']),rejection=None,latency_ms=1)
log.write('action',requested='STOP',accepted='STOP')
log.write('action_result',requested='STOP',applied='STOP',result='completed')
log.write('end',outcome='returned_empty')
log.file.close()
(directory/'ground_truth.jsonl').write_text('{}\\n')
'''
                def launch(command, env, output, timeout):
                    if brain in ('laya', 'kev'):
                        with runtime.open('a') as local:
                            local.write('new inference diagnostics\n')
                    decision = {'choice': 'STOP', 'raw_response': raw, 'payload': {'state': 'private prompt'}}
                    return run_webots([sys.executable, '-u', '-c', script], {**env, 'TEST_DECISION': json.dumps(decision)}, output, timeout)
                with patch.object(run_experiment, 'ROOT', root), \
                     patch.object(run_experiment, 'find_webots', return_value=sys.executable), \
                     patch.object(run_experiment.subprocess, 'run', return_value=SimpleNamespace(stdout='R2025a')), \
                     patch.object(run_experiment, 'run_webots', side_effect=launch), \
                     patch.object(run_experiment, 'summarize', return_value={'valid': True, 'outcome': 'returned_empty'}), \
                     patch('mission.brain.build_opener', return_value=opener), \
                     patch.dict(os.environ, {'OPENROUTER_API_KEY': 'secret-placeholder', 'MARS_TEST_DELAY_S': '0'}), \
                     patch.object(sys, 'argv', ['run_experiment.py', '--brain', brain, '--world', str(world)]):
                    run_experiment.main()
                text = next((root / 'results').glob('*/webots.log')).read_text()
                blocks = text.split(OUTPUT_SEPARATOR)[1::2]
                self.assertEqual(len(blocks), 2)
                self.assertTrue(all(f'{brain.upper()} OUTPUT' in block for block in blocks))
                self.assertIn('HTTP 200', blocks[0])
                self.assertIn('warmup', blocks[0])
                self.assertTrue(all(json.loads(block[block.index('{'):]) == raw for block in blocks))
                self.assertIn('\n  "answers": {\n    "mission": {', text)
                self.assertNotIn('private prompt', text)
                self.assertNotIn('secret-placeholder', text)
                self.assertNotIn('previous experiment', text)
                self.assertNotIn('inference diagnostics', text)
                self.assertNotIn('action_result', text)
                self.assertNotIn('summary', text)
                steps = next((root / 'results').glob('*/steps.jsonl')).read_text()
                self.assertIn('action_result', steps)
                self.assertIn('private prompt', steps)

    def test_warmup_failure_stays_in_warmup_json_without_polluting_webots_log(self):
        with TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            root = Path(directory)
            world = root / 'mars.wbt'
            world.write_text('# test world')
            with patch.object(run_experiment, 'ROOT', root), \
                 patch.object(run_experiment, 'find_webots', return_value=sys.executable), \
                 patch.object(Brain, 'decide', side_effect=RuntimeError('secret-error-body')), \
                 patch.object(sys, 'argv', ['run_experiment.py', '--brain', 'laya', '--world', str(world)]):
                with self.assertRaisesRegex(SystemExit, 'Brain warmup failed'):
                    run_experiment.main()
            text = next((root / 'results').glob('*/webots.log')).read_text()
            self.assertEqual(text, '')
            error = next((root / 'results').glob('*/warmup.json')).read_text()
            self.assertEqual(json.loads(error)['error_type'], 'RuntimeError')
            self.assertNotIn('secret-error-body', error)

    def test_action_panel_scrolls_and_updates_in_place(self):
        with patch.dict(sys.modules, {'controller': SimpleNamespace(Supervisor=Mock)}):
            from controllers.mission_supervisor.mission_supervisor import show_actions
        labels = {}
        robot = Mock()
        robot.setLabel.side_effect = lambda label, text, *args: labels.update({label: text})
        actions = deque(maxlen=6)
        for i in range(1, 9):
            action = ActionExecution('FORWARD', 'FORWARD', None, i, [0, 0], 0)
            data = {'request_id': i, **action.entry}
            show_actions(robot, actions, data)
            action.finish('front_obstacle', safety='front_obstacle')
            show_actions(robot, actions, {'request_id': i, **action.entry})
        self.assertEqual([entry['request_id'] for entry in actions], list(range(3, 9)))
        self.assertIn('#3 3.0s FORWARD', labels[10])
        self.assertIn('#8 8.0s FORWARD', labels[20])
        self.assertEqual(labels[21], '-> STOP | blocked: front_obstacle')
        self.assertEqual(len(labels), 12)
        fallback = ActionExecution('FORWARD', 'STOP', 'state_changed', 9, [0, 0], 0)
        fallback.finish('interval_elapsed')
        show_actions(robot, actions, {'request_id': 9, **fallback.entry})
        self.assertEqual(labels[21], '-> STOP | completed: state_changed')
        self.assertEqual(robot.setLabel.call_args.args[5], 0xFF8080)

    def test_console_responses_include_jev_without_prompt_payload(self):
        with TemporaryDirectory() as directory, redirect_stdout(io.StringIO()) as console:
            log = Log(directory, brain='jev')
            decision = {'choice': 'FORWARD', 'confidence': .9, 'probabilities': {'FORWARD': 1},
                        'served_model': 'jev', 'payload': {'state': 'full prompt'}}
            raw = {'model': 'jev', 'answers': {'mission': {'type': 'choice', 'choice': 'FORWARD',
                   'confidence': .9, 'probabilities': {'FORWARD': 1}}}, 'usage': {'input_tokens': 1033, 'output_tokens': 0}}
            decision.update(raw_response=raw, http_status=200)
            log.write('request', request_id=3, payload={'state': 'full prompt'})
            self.assertEqual(console.getvalue(), '')
            log.write('response', simulation_s=1, request_id=3, decision=decision, rejection=None, latency_ms=50)
            log.write('late_response', simulation_s=2, request_id=4, decision=decision, rejection='already_rejected', latency_ms=900)
            log.write('action_result', request_id=3, requested='FORWARD', applied='STOP', result='blocked')
            log.file.close()
            blocks = console.getvalue().split(OUTPUT_SEPARATOR)[1::2]
            self.assertEqual(len(blocks), 2)
            self.assertTrue(all(json.loads(block[block.index('{'):]) == raw for block in blocks))
            self.assertIn('JEV OUTPUT | HTTP 200 | 50.0 ms', blocks[0])
            self.assertIn('  >>> mission: FORWARD | confidence=0.9', blocks[0])
            self.assertIn('late response; ignored', blocks[1])
            self.assertNotIn('action_result', console.getvalue())
            self.assertNotIn('request_id', console.getvalue())
            self.assertNotIn('full prompt', console.getvalue())
            self.assertIn('full prompt', (Path(directory) / 'steps.jsonl').read_text())

    def test_launch_streams_before_exit_and_keeps_output_on_timeout(self):
        with TemporaryDirectory() as directory:
            gate = Path(directory) / 'seen'
            # The child only exits after the parent has shown its first line.
            script = 'import sys,time; from pathlib import Path; from mission.logging import brain_output; print("runtime noise",flush=True); print(brain_output("jev", {"model":"jev","answers":{}}),end="",flush=True)\nwhile not Path(sys.argv[1]).exists(): time.sleep(.01)\nprint("runtime finished",flush=True)'
            shown = []
            def display(line, **kwargs):
                shown.append(line)
                gate.touch()
            with (Path(directory) / 'webots.log').open('w+') as output, patch('run_experiment.print', side_effect=display, create=True):
                self.assertEqual(run_webots([sys.executable, '-u', '-c', script, str(gate)], os.environ.copy(), output, 3), 0)
                output.seek(0)
                self.assertEqual(output.read(), ''.join(shown))
                self.assertNotIn('runtime noise', ''.join(shown))
                self.assertIn('\n  "model": "jev",', ''.join(shown))
                self.assertIn('runtime noise', (Path(directory) / 'webots-runtime.log').read_text())
            with (Path(directory) / 'timeout.log').open('w+') as output, redirect_stdout(io.StringIO()):
                with self.assertRaises(subprocess.TimeoutExpired):
                    run_webots([sys.executable, '-u', '-c', 'import time; from mission.logging import brain_output; print(brain_output("jev", {"model":"jev","answers":{}}),end="",flush=True); time.sleep(5)'], os.environ.copy(), output, .5)
                output.seek(0)
                self.assertIn('JEV OUTPUT', output.read())

    def test_drive_tripled_and_slow_forward_removed_from_contract(self):
        with patch.dict(sys.modules, {'controller': SimpleNamespace(Robot=Mock)}):
            from controllers.rover.rover import Drive
        devices = {}
        robot = Mock()
        robot.getDevice.side_effect = lambda name: devices.setdefault(name, Mock())
        drive = Drive(robot)
        self.assertEqual(drive.command('forward', 0)['wheel_rad_s'], {'Left': -1.8, 'Right': -1.8})
        reverse = drive.command('reverse', 0)
        self.assertEqual(reverse['wheel_rad_s'], {'Left': 1.8, 'Right': 1.8})
        self.assertTrue(all(angle == 0 for angle in reverse['steering'].values()))
        self.assertEqual(drive.command('spin', 1)['wheel_rad_s'], {'Left': 1.8, 'Right': -1.8})
        self.assertEqual(drive.command('spin', -1)['wheel_rad_s'], {'Left': -1.8, 'Right': 1.8})
        self.assertEqual(drive.command('stop', 0)['wheel_rad_s'], {'Left': 0, 'Right': 0})
        for name in devices:
            if name.endswith('Wheel'):
                devices[name].setVelocity.assert_called_with(0)
        o = observation()
        self.assertNotIn('SLOW_FORWARD', criteria(o))
        o.update(candidates=[], goal={'kind': 'exploration'}, rules_target={'bearing_deg': 0, 'distance_m': .7})
        self.assertEqual(rules(o), 'FORWARD')
        root = Path(__file__).resolve().parents[1]
        self.assertEqual((root / 'protos/Sojourner.proto').read_text().count('maxVelocity 1.8'), 6)
        self.assertIn('EXTERNPROTO "../protos/Sojourner.proto"', (root / 'worlds/mars.wbt').read_text())

    def test_pending_worker_keeps_original_motor_and_history_snapshot(self):
        action = ActionExecution('FORWARD', 'FORWARD', None, 1, [0, 0], 0)
        o = observation()
        o['history'] = execution_history([action.entry])
        gate = Event()
        class DelayedReader:
            def decide(self, state):
                gate.wait(1)
                return {'choice': 'STOP', 'original_history': state['history']}
        pending = Pending(DelayedReader(), o, 1)
        action.command({'mode': 'forward', 'wheel_rad_s': {'Left': -.6, 'Right': -.6}})
        action.observe(1.5, [.02, 0], 0)
        action.finish('front_obstacle', safety='front_obstacle')
        gate.set()
        pending.thread.join(1)
        original = pending.poll(o, 1)['decision']['original_history'][0]
        self.assertEqual(original['result'], 'running')
        self.assertEqual(original['applied'], 'STOP')
        self.assertEqual(original['motor']['wheel_rad_s']['Left'], 0)
        self.assertEqual(original['displacement_m'], 0)

    def test_blocked_forward_reports_stop_without_restricting_model_choices(self):
        o = observation()
        choices = criteria(o)
        o['lidar_m']['front'] = .2
        o['safety'] = 'front_obstacle'
        action = ActionExecution('FORWARD', 'FORWARD', None, 1, [0, 0], 0)
        action.command({'mode': 'stop', 'wheel_rad_s': {'Left': 0, 'Right': 0}})
        action.finish('front_obstacle', safety='front_obstacle')
        o['history'] = [action.entry]
        self.assertEqual({key: value for key, value in criteria(o).items() if key != 'REVERSE'}, choices)
        self.assertIn('REVERSE', criteria(o))
        self.assertEqual(action.entry['requested'], 'FORWARD')
        self.assertEqual(action.entry['applied'], 'STOP')
        self.assertEqual(action.entry['result'], 'blocked')
        self.assertEqual(action.entry['executed_s'], 0)
        self.assertEqual(action.entry['displacement_m'], 0)
        for name in ('laya', 'kev', 'jev'):
            with patch.dict('os.environ', {'OPENROUTER_API_KEY': 'test-placeholder'}):
                payload = Brain(name).payload(o)
                self.assertEqual(payload['questions']['mission']['criteria'], criteria(o))
            self.assertIn('"result":"blocked"', payload['state'])

    def test_reverse_repositions_after_blocked_turn_and_stops_at_rear_hazards(self):
        o = observation()
        o['lidar_m'].update(front=.237, rear=1.622)
        o['safety'] = 'spin_clearance'
        blocked = ActionExecution('TURN_LEFT', 'TURN_LEFT', None, 1, [0, 0], 0)
        blocked.finish('spin_clearance', safety='spin_clearance')
        o['history'] = execution_history([blocked.entry])
        self.assertEqual(safety([0, 0, .2], 0, 0, o['lidar_m'], 'spin'), 'spin_clearance')
        self.assertIsNone(safety([0, 0, .2], 0, 0, o['lidar_m'], 'reverse'))
        with patch.dict(os.environ, {'OPENROUTER_API_KEY': 'test-placeholder'}):
            for name in ('laya', 'kev', 'jev'):
                payload = Brain(name).payload(o)
                question = payload['questions']['mission']
                self.assertIn('REVERSE', question['criteria'])
                self.assertIn('REVERSE is offered only after the last movement was blocked', question['instructions'])
                self.assertIn('Do not repeat blocked turns', question['instructions'])
                raw = {'model': Brain(name).model, 'answers': {'mission': {'type': 'choice', 'choice': 'REVERSE',
                       'probabilities': {k: int(k == 'REVERSE') for k in question['criteria']}}}}
                self.assertEqual(Brain(name).decode(raw, o)['choice'], 'REVERSE')
        action = ActionExecution('REVERSE', 'REVERSE', None, 1, [0, 0], 0)
        action.command({'mode': 'reverse', 'wheel_rad_s': {'Left': 1.8, 'Right': 1.8}})
        action.observe(1.8, [-.086, 0], 0)
        action.command({'mode': 'stop', 'wheel_rad_s': {'Left': 0, 'Right': 0}})
        action.finish('interval_elapsed')
        self.assertEqual(action.entry['applied'], 'REVERSE')
        self.assertEqual(action.entry['result'], 'completed')
        self.assertEqual(action.entry['displacement_m'], .086)
        self.assertEqual(action.entry['motor']['mode'], 'stop')
        o['lidar_m']['rear'] = .7
        self.assertEqual(safety([0, 0, .2], 0, 0, o['lidar_m'], 'reverse'), 'rear_obstacle')
        self.assertIn('REVERSE', criteria(o))
        self.assertEqual(safety([-.7, 0, .2], 0, 0, o['lidar_m'], 'reverse'), 'boundary_behind')
        self.assertEqual(safety([3.3, 0, .2], 0, 0, o['lidar_m'], 'reverse', yaw=math.pi), 'boundary_behind')

    def test_reverse_requires_the_last_movement_to_be_blocked(self):
        o = observation()
        self.assertNotIn('REVERSE', criteria(o))
        o['safety'] = 'front_obstacle'
        o['last_safety_intervention'] = {'reason': 'front_obstacle', 'simulation_s': 0}
        o['lidar_m']['front'] = .2
        o['progress']['no_progress'] = True
        self.assertNotIn('REVERSE', criteria(o))
        blocked = ActionExecution('FORWARD', 'FORWARD', None, 1, [0, 0], 0)
        blocked.finish('front_obstacle', safety='front_obstacle')
        o['history'] = execution_history([blocked.entry])
        self.assertIn('REVERSE', criteria(o))
        stop = ActionExecution('STOP', 'STOP', None, 2, [0, 0], 0)
        stop.finish('interval_elapsed')
        o['history'] = execution_history([blocked.entry, stop.entry])
        self.assertIn('REVERSE', criteria(o))
        for movement in ('REVERSE', 'FORWARD', 'TURN_LEFT', 'TURN_RIGHT'):
            completed = ActionExecution(movement, movement, None, 3, [0, 0], 0)
            completed.finish('interval_elapsed')
            o['history'] = execution_history([blocked.entry, completed.entry])
            self.assertNotIn('REVERSE', criteria(o))
        interrupted = ActionExecution('FORWARD', 'FORWARD', None, 4, [0, 0], 0)
        interrupted.command({'mode': 'forward', 'wheel_rad_s': {'Left': -1.8, 'Right': -1.8}})
        interrupted.observe(4.2, [.02, 0], 0)
        interrupted.finish('front_obstacle', safety='front_obstacle')
        o['history'] = execution_history([interrupted.entry])
        self.assertIn('REVERSE', criteria(o))
        interrupted.finish('superseded')
        o['history'] = execution_history([interrupted.entry])
        self.assertNotIn('REVERSE', criteria(o))
        fallback = ActionExecution('FORWARD', 'STOP', 'state_changed', 5, [0, 0], 0)
        fallback.finish('interval_elapsed')
        o['history'] = execution_history([fallback.entry])
        self.assertNotIn('REVERSE', criteria(o))
        o.update(history=execution_history([blocked.entry]), sensors_valid=False)
        self.assertNotIn('REVERSE', criteria(o))

    def test_partial_execution_and_commanded_motion_have_measured_results(self):
        action = ActionExecution('FORWARD', 'FORWARD', None, 1, [0, 0], 0)
        action.command({'mode': 'forward', 'wheel_rad_s': {'Left': -.6, 'Right': -.6}})
        action.observe(1.5, [.02, 0], 0)
        action.command({'mode': 'stop', 'wheel_rad_s': {'Left': 0, 'Right': 0}})
        action.finish('front_obstacle', safety='front_obstacle')
        self.assertEqual(action.entry['result'], 'interrupted')
        self.assertEqual(action.entry['executed_s'], .5)
        self.assertEqual(action.entry['displacement_m'], .02)
        self.assertEqual(action.entry['applied'], 'FORWARD')
        self.assertEqual(action.entry['motor']['mode'], 'stop')
        stuck = ActionExecution('TURN_LEFT', 'TURN_LEFT', None, 1, [0, 0], math.pi-.02)
        stuck.command({'mode': 'spin', 'wheel_rad_s': {'Left': .6, 'Right': -.6}})
        stuck.observe(1.8, [0, 0], math.pi-.02)
        self.assertEqual(stuck.entry['displacement_m'], 0)
        self.assertEqual(stuck.entry['yaw_change_deg'], 0)
        stuck.observe(1.9, [0, 0], -math.pi+.02)
        self.assertAlmostEqual(stuck.entry['yaw_change_deg'], 2.29, places=2)

    def test_history_records_physical_operation_ack_and_fallback(self):
        for ok in (False, True):
            action = ActionExecution('COLLECT_S1', 'COLLECT_S1', None, 1, [0, 0], 0)
            action.observe(2.2, [0, 0], 0)
            action.finish('physical_validation', operation_ok=ok)
            self.assertEqual(action.entry['applied'], 'COLLECT_S1' if ok else 'STOP')
            self.assertEqual(action.entry['result'], 'completed' if ok else 'failed')
            self.assertEqual(action.entry['motor']['mode'], 'stop')
        action = ActionExecution('FORWARD', 'STOP', 'state_changed', 1, [0, 0], 0)
        action.observe(1.8, [0, 0], 0)
        action.finish('interval_elapsed')
        self.assertEqual(action.entry['applied'], 'STOP')
        self.assertEqual(action.entry['fallback'], 'state_changed')

    def test_candidate_contract_and_response_validation(self):
        o = observation()
        self.assertEqual(rules(o), 'TURN_RIGHT')
        self.assertNotIn('INSPECT_S1', criteria(o))
        self.assertNotIn('COLLECT_S1', criteria(o))
        self.assertIn('RETURN_TO_BASE', criteria(o))
        o['goal']['kind'] = 'base'
        self.assertNotIn('RETURN_TO_BASE', criteria(o))
        o['goal']['kind'] = 'sample'
        o['candidates'][0].update(distance_m=.6, bearing_deg=0, inspect_ready=True)
        self.assertIn('INSPECT_S1', criteria(o))
        brain = Brain('laya')
        keys = criteria(o)
        raw = {'model': 'laya', 'answers': {'mission': {'type': 'choice', 'choice': 'INSPECT_S1', 'confidence': .8,
                                                      'probabilities': {k: int(k == 'INSPECT_S1') for k in keys}}}}
        self.assertEqual(brain.decode(raw, o)['choice'], 'INSPECT_S1')
        raw['answers']['mission']['choice'] = 'COLLECT_S3'
        with self.assertRaises(ValueError):
            brain.decode(raw, o)
        raw['answers']['mission']['choice'] = 'INSPECT_S1'
        raw['answers']['mission']['probabilities']['STOP'] = float('nan')
        with self.assertRaises(ValueError):
            brain.decode(raw, o)
        o['hidden_samples'] = {'SECRET_POSITION': [2, 3]}
        o['rules_target'] = {'SECRET_ROUTE': [1, 2]}
        o['rules_navigation'] = {'SECRET_PATH': [1, 2]}
        payload = json.dumps(brain.payload(o))
        for forbidden in ('S2', 'SECRET_POSITION', 'SECRET_ROUTE', 'SECRET_PATH', 'collect 3', 'all 3'):
            self.assertNotIn(forbidden, payload)
        with patch.dict('os.environ', {'LAYA_BASE_URL': 'https://example.com'}):
            with self.assertRaises(ValueError):
                Brain('laya')

    def test_return_is_independent_of_known_inventory(self):
        o = observation()
        o.update(candidates=[], collected=['S1'], goal={'kind': 'exploration'})
        self.assertNotEqual(rules(o), 'RETURN_TO_BASE')
        o['return_due'] = True
        self.assertEqual(rules(o), 'RETURN_TO_BASE')

    def test_model_adapters_share_mission_contract_without_calls(self):
        with patch.dict('os.environ', {'OPENROUTER_API_KEY': 'test-placeholder'}):
            brains = [Brain(name) for name in ('laya', 'kev', 'jev')]
            payloads = [brain.payload(observation()) for brain in brains]
            for payload in payloads[1:]:
                self.assertEqual(payload['state'], payloads[0]['state'])
                self.assertEqual(payload['questions'], payloads[0]['questions'])
                self.assertNotIn('test-placeholder', json.dumps(payload))

    def test_deadline_revision_and_age_independence(self):
        gate = Event()
        class Slow:
            def decide(self, o):
                gate.wait(.5)
                return {'choice': 'FORWARD'}
        p = Pending(Slow(), observation(), 1)
        self.assertIsNone(p.poll(observation(), 1))
        self.assertTrue(p.thread.is_alive())
        with patch('mission.brain.perf_counter', return_value=p.started + 2):
            self.assertEqual(p.poll(observation(), 1)['rejection'], 'deadline_exceeded')
        self.assertIsNone(p.poll(observation(), 1))
        gate.set()
        p.thread.join(1)
        for now, revision, expected in ((1, 3, 'state_changed'), (500, 1, None)):
            p = Pending(Brain('rules'), observation(), 2)
            p.thread.join(1)
            current = observation()
            current.update(simulation_s=now, revision=revision)
            self.assertEqual(p.poll(current, 1)['rejection'], expected)

    def test_windows_setup_uses_exe_and_relative_paths(self):
        def download(url, target):
            target.parent.mkdir(parents=True, exist_ok=True)
            if target.suffix == '.zip':
                with zipfile.ZipFile(target, 'w') as archive:
                    archive.writestr('bin/llama-server.exe', b'test')
            else:
                target.write_bytes(b'test weights')
        with TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.object(setup_models, 'ROOT', root), patch.object(setup_models, 'MODEL_DIR', root / '.local_models'), patch.object(setup_models.platform, 'system', return_value='Windows'), patch.object(setup_models.platform, 'machine', return_value='AMD64'), patch.object(setup_models, 'download', side_effect=download):
                manifest = setup_models.setup(originals=False, brains=('laya',))
                executable = manifest['runtime']['executable']
                self.assertTrue(executable.endswith('llama-server.exe'))
                self.assertFalse(Path(executable).is_absolute())
                self.assertTrue((root / executable).is_file())
                self.assertEqual(len(manifest['artifacts']), 1)

    def test_pose_and_clearance_invalidate_a_response_without_inventory_change(self):
        for changes, expected in (({'pose': [.1, 0, 0]}, 'pose_changed'),
                                  ({'pose': [0, 0, .2]}, 'pose_changed'),
                                  ({'lidar_m': {'front': .2, 'left': 4, 'right': 4, 'rear': 4}}, 'clearance_changed')):
            p = Pending(Brain('rules'), observation(), 2)
            p.thread.join(1)
            current = observation()
            current.update(changes)
            self.assertEqual(p.poll(current, 1)['rejection'], expected)

    def test_error_text_never_logged(self):
        class Broken:
            def decide(self, o):
                raise RuntimeError('secret-key-123')
        p = Pending(Broken(), observation(), 1)
        p.thread.join(1)
        result = p.poll(observation(), 1)
        self.assertNotIn('secret-key', json.dumps(result))
        self.assertEqual(result['rejection'], 'RuntimeError')

    def test_independent_baseline_routes_around_observed_obstacle(self):
        baseline = Navigator()
        baseline.obstacles = {(7, 0), (7, 1), (7, -1)}
        route = baseline.plan([0, 0], [2, 0])
        self.assertTrue(route)
        self.assertTrue(any(abs(y) > .5 for x, y in route))

    def test_sensor_sectors_coverage_and_safety(self):
        sectors = lidar_summary([(.2, 0, 0), (0, 1, 0), (0, -2, 0), (-.38, 0, 0), (math.inf, math.inf, 0)])
        self.assertEqual(sectors, {'front': .2, 'left': 1.0, 'right': 2.0, 'rear': 4.0})
        immediate = lidar_summary([(.05, 0, 0)])
        self.assertEqual(safety([0, 0, .2], 0, 0, immediate, 'forward'), 'front_obstacle')
        self.assertEqual(safety([0, 0, .2], 0, 0, sectors, 'forward'), 'front_obstacle')
        self.assertEqual(safety([0, 0, .2], .5, 0, sectors, 'stop'), 'excessive_tilt')
        self.assertEqual(safety([3.4, 0, .2], 0, 0, {k:4 for k in sectors}, 'forward'), 'boundary_ahead')
        self.assertAlmostEqual(relative([0, 0], 0, [0, -1])['bearing_deg'], -90)
        coverage = Coverage()
        coverage.observe([0, 0])
        first = coverage.state([0, 0], 0)['visited_pct']
        coverage.observe([0, 0])
        self.assertEqual(first, coverage.state([0, 0], 0)['visited_pct'])
        coverage.observe([2, 1])
        self.assertGreater(coverage.state([2, 1], 0)['visited_pct'], first)


if __name__ == '__main__':
    unittest.main()
