"""Audit real Webots run evidence: python3 tests/verify_physical.py RUN_DIR [--rejection REASON]."""
import argparse
import json
import math
import struct
from pathlib import Path


def verify(directory, rejection=None):
    records = [json.loads(line) for line in (directory / 'steps.jsonl').read_text().splitlines()]
    truth = [json.loads(line) for line in (directory / 'ground_truth.jsonl').read_text().splitlines()]
    assert records[-1]['type'] == 'end' and truth[-1]['type'] == 'end', 'Both writers must finish'
    end = records[-1]
    assert end.get('stop_flushed'), 'Final motor stop must reach Webots'
    steps = [r for r in records if r['type'] == 'step']
    assert steps and all(b['simulation_s'] > a['simulation_s'] for a, b in zip(steps, steps[1:]))
    for step in steps:
        assert all(math.isfinite(v) for v in (*step['position'], step['roll'], step['pitch'], step['yaw']))
        assert max(abs(step['roll']), abs(step['pitch'])) < .4, 'Rover tilted too far'
        if step['safety']:
            assert step['applied']['mode'] == 'stop'
    seen = set()
    requests = {}
    for r in records:
        if r['type'] == 'detected':
            seen.add(r['sample'])
        if r['type'] == 'request':
            o = r['observation']
            assert {c['id'] for c in o['candidates']} <= seen, 'Hidden sample leaked'
            requests[r['request_id']] = r
            frame = o['frame']
            data = (directory / frame['path']).read_bytes()
            assert data[:8] == b'\x89PNG\r\n\x1a\n'
            assert struct.unpack('>II', data[16:24]) == (320, 240)
            assert abs(frame['simulation_s'] - o['simulation_s']) < 1e-6, 'Frame must be current'
        if r['type'] == 'response':
            assert r['request_id'] in requests
            if r['rejection']:
                actions = [a for a in records if a['type'] == 'action' and a['request_id'] == r['request_id']]
                assert actions and actions[0]['fallback'] == r['rejection']
    accepted_operations = [r for r in truth if r['type'] == 'operation' and r['ok']]
    inspected = set()
    for operation in accepted_operations:
        assert operation['simulation_s'] - operation['stationary_since'] >= 1
        if operation['event'] in ('inspect', 'collect'):
            assert operation['distance_m'] <= .68
            if operation['event'] == 'inspect':
                inspected.add(operation['sample'])
            else:
                assert operation['sample'] in inspected
        else:
            assert math.hypot(*operation['physical_position'][:2]) <= .3
    if rejection:
        responses = [r for r in records if r['type'] == 'response' and r['rejection'] == rejection]
        assert responses, 'Expected rejected inference missing'
        for response in responses:
            request = requests[response['request_id']]
            late = next((r for r in records if r['type'] == 'late_response' and r['request_id'] == response['request_id']), response)
            during = [s for s in steps if request['observation']['simulation_s'] <= s['simulation_s'] <= late['simulation_s']]
            assert during, 'Physics did not advance during inference'
        if rejection == 'deadline_exceeded':
            assert any(s.get('pending_reported') and s['applied']['mode'] != 'stop' for s in steps), 'Fallback must move while timed-out thread still occupies slot'
            assert any(r['type'] == 'late_response' for r in records), 'Late response must be discarded and logged'
    elif end['outcome'] == 'success':
        assert set(end['delivered']) == {'S1', 'S2', 'S3'}
        assert sum(o['event'] == 'collect' for o in accepted_operations) == 3
        assert sum(o['event'] == 'deliver' for o in accepted_operations) == 1
        assert end['path_length_m'] > 1
        assert not any(r['obstacle_contacts'] for r in truth if r['type'] == 'trajectory')
    elif end['outcome'] == 'sensors_complete':
        measured = next(r for r in records if r['type'] == 'sensor_result')
        assert measured['image_bytes'] == 320 * 240 * 4
        assert measured['lidar_finite_returns'] > 0 and measured['observed']
        assert all(math.isfinite(v) for v in measured['compass'])
        assert (directory / 'sensors.png').is_file()
    elif end['outcome'] == 'motion_complete':
        stage = lambda t: min(steps, key=lambda s: abs(s['simulation_s'] - t))
        assert stage(12)['position'][0] - stage(2)['position'][0] > .25, 'Forward axis incorrect'
        assert stage(24)['yaw'] > stage(12)['yaw'] + .2, 'Left steering not observed'
        assert abs(stage(38)['yaw'] - stage(24)['yaw']) > .4, 'Spin not observed'
        assert stage(40)['speed_m_s'] < .006, 'Stop not observed'
    print(json.dumps({'run': directory.name, 'outcome': end['outcome'], 'checked_frames': len(requests),
                      'physical_operations': len(accepted_operations), 'rejection_checked': rejection}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--rejection')
    args = parser.parse_args()
    verify(args.directory, args.rejection)
