import json
import math
import unittest
from unittest.mock import patch
from threading import Event
from pathlib import Path
from tempfile import TemporaryDirectory
import zipfile
import setup_models
from mission.brain import Brain, Pending, criteria, rules
from mission.navigation import Navigator, safety


def observation():
    return {'simulation_s': 1, 'revision': 2, 'battery_pct': 100, 'collected': [], 'delivered': [],
            'candidates': [{'id': 'S1', 'distance_m': 1, 'inspected': False}], 'sensors_valid': True, 'obstacle': False, 'history': []}


class Checks(unittest.TestCase):
    def test_candidate_contract_and_response_validation(self):
        o = observation()
        self.assertEqual(rules(o), 'INSPECT_S1')
        self.assertNotIn('COLLECT_S1', criteria(o))
        brain = Brain('laya')
        keys = criteria(o)
        raw = {'model': 'laya', 'answers': {'mission': {'type': 'choice', 'choice': 'INSPECT_S1', 'confidence': .8,
                                                      'probabilities': {k: int(k == 'INSPECT_S1') for k in keys}}}}
        self.assertEqual(brain.decode(raw, o)['choice'], 'INSPECT_S1')
        raw['answers']['mission']['choice'] = 'COLLECT_S3'
        with self.assertRaises(ValueError):
            brain.decode(raw, o)
        self.assertNotIn('S2', json.dumps(brain.payload(o)))
        with patch.dict('os.environ', {'LAYA_BASE_URL': 'https://example.com'}):
            with self.assertRaises(ValueError):
                Brain('laya')

    def test_model_adapters_share_mission_contract_without_calls(self):
        with patch.dict('os.environ', {'OPENROUTER_API_KEY': 'test-placeholder'}):
            brains = [Brain(name) for name in ('laya', 'kev', 'jev')]
            payloads = [brain.payload(observation()) for brain in brains]
            for payload in payloads[1:]:
                self.assertEqual(payload['state'], payloads[0]['state'])
                self.assertEqual(payload['questions'], payloads[0]['questions'])
                self.assertNotIn('test-placeholder', json.dumps(payload))

    def test_deadline_revision_and_age_rejections(self):
        gate = Event()
        class Slow:
            def decide(self, o):
                gate.wait(.5)
                return {'choice': 'INSPECT_S1'}
        p = Pending(Slow(), observation(), 1)
        self.assertIsNone(p.poll(1, 2, 1, 3))
        self.assertTrue(p.thread.is_alive())
        with patch('mission.brain.perf_counter', return_value=p.started + 2):
            self.assertEqual(p.poll(1, 2, 1, 3)['rejection'], 'deadline_exceeded')
        self.assertIsNone(p.poll(1, 2, 1, 3))
        gate.set()
        p.thread.join(1)
        for now, revision, expected in ((1, 3, 'state_changed'), (5, 2, 'stale_observation')):
            p = Pending(Brain('rules'), observation(), 2)
            p.thread.join(1)
            self.assertEqual(p.poll(now, revision, 1, 3)['rejection'], expected)

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

    def test_final_waypoint_is_reachable(self):
        nav = Navigator()
        mode, _ = nav.command([.49, -.32, .2], -.7, (.56, -.46), 1)
        self.assertNotEqual(mode, 'no_route')
        nav.observe([0, 0, .2], 0, [(-.6, .15, 0)], [])
        self.assertEqual(nav.obstacles, set())

    def test_error_text_never_logged(self):
        class Broken:
            def decide(self, o):
                raise RuntimeError('secret-key-123')
        p = Pending(Broken(), observation(), 1)
        p.thread.join(1)
        result = p.poll(1, 2, 1, 3)
        self.assertNotIn('secret-key', json.dumps(result))
        self.assertEqual(result['rejection'], 'RuntimeError')

    def test_route_avoids_observed_obstacle_and_safety_stops(self):
        nav = Navigator()
        nav.obstacles = {(7, 0), (7, 1), (7, -1)}
        route = nav.plan([0, 0], [2, 0])
        self.assertTrue(route)
        self.assertTrue(any(abs(y) > .5 for x, y in route))
        ranges = [math.inf] * 360
        self.assertIsNone(safety([0, 0, .2], 0, 0, ranges, 'forward'))
        ranges[180] = .1
        self.assertEqual(safety([0, 0, .2], 0, 0, ranges, 'forward'), 'front_obstacle')
        self.assertEqual(safety([0, 0, .2], .5, 0, ranges, 'stop'), 'excessive_tilt')


if __name__ == '__main__':
    unittest.main()
