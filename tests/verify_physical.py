"""Audit Webots evidence: python3 tests/verify_physical.py RUN_DIR [--rejection REASON] [--blocked]."""
import argparse
import json
import math
import struct
from pathlib import Path


def verify(directory, rejection=None, blocked=False):
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
        if (step.get('active') or '').startswith(('INSPECT_', 'COLLECT_')):
            assert step['applied']['mode'] == 'stop', 'Interaction must not approach automatically'
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
                if r['rejection'] != 'mission_ended':
                    assert actions and actions[0]['fallback'] == r['rejection'] and actions[0].get('accepted', actions[0].get('applied')) == 'STOP'
    results = {r['started_s']: r for r in records if r['type'] == 'action_result'}
    for r in records:
        if r['type'] == 'action':
            accepted = r.get('accepted', r.get('applied'))
            assert 0 < r['duration_s'] <= (3.01 if accepted.startswith(('INSPECT_', 'COLLECT_')) else 1.01)
            assert accepted in ('FORWARD', 'REVERSE', 'SLOW_FORWARD', 'TURN_LEFT', 'TURN_RIGHT', 'STOP', 'RETURN_TO_BASE') or accepted.startswith(('INSPECT_', 'COLLECT_'))
            if 'accepted' in r:
                outcome = results[r['simulation_s']]
                assert outcome['accepted'] == accepted and outcome['requested'] == r['requested']
                assert outcome['motor']['mode'] == 'stop', 'Finished action must command braking'
                assert outcome['ended_s'] >= outcome['started_s']
                if outcome['result'] == 'blocked':
                    assert outcome['safety'] and outcome['applied'] == 'STOP'
                    assert outcome['executed_s'] == 0 and outcome['displacement_m'] == 0 and outcome['yaw_change_deg'] == 0
                    assert all(v == 0 for v in outcome['motor']['wheel_rad_s'].values())
            if r['source'] == 'fallback':
                assert accepted == 'STOP'
        if r['type'] == 'request' and r.get('payload'):
            state = r['payload']['state']
            assert 'total_samples' not in state and 'undiscovered' not in state and 'rules_target' not in state
            o = r['observation']
            if o['sensors_valid']:
                assert {'FORWARD', 'TURN_LEFT', 'TURN_RIGHT', 'STOP'} <= set(r['payload']['questions']['mission']['criteria']), 'Protection must not restrict model choices'
        if r['type'] == 'request':
            for item in r['observation']['history']:
                if 'started_s' in item:
                    outcome = results[item['started_s']]
                    assert all(outcome[key] == value for key, value in item.items()), 'Next inference must see the finalized execution, not acceptance'
        if r['type'] == 'step' and r['applied']['mode'] != 'stop' and records[0]['test'] == 'mission':
            assert r['active'] in ('FORWARD', 'REVERSE', 'SLOW_FORWARD', 'TURN_LEFT', 'TURN_RIGHT'), 'Unrequested navigation'
    accepted_operations = [r for r in truth if r['type'] == 'operation' and r['ok']]
    inspected = set()
    for operation in accepted_operations:
        assert operation['simulation_s'] - operation['stationary_since'] >= 1
        if operation['event'] in ('inspect', 'collect'):
            assert operation['distance_m'] <= .68
            if operation['event'] == 'inspect':
                assert operation['visible'] and abs(operation['bearing_deg']) <= 20
                inspected.add(operation['sample'])
            else:
                assert operation['sample'] in inspected
        else:
            assert math.hypot(*operation['physical_position'][:2]) <= .3
    physically_collected = {r['sample'] for r in accepted_operations if r['event'] == 'collect'}
    assert set(end.get('collected', [])) == physically_collected, 'Inventory must follow physical ACKs'
    assert all(set(step['collected']) <= physically_collected for step in steps)
    if blocked:
        assert any(r['source'] == 'model' and r['requested'] == 'FORWARD' and r['result'] == 'blocked'
                   for r in results.values()), 'Expected a real model FORWARD blocked by protection'
        assert any(h.get('result') == 'blocked' and h['applied'] == 'STOP'
                   for r in requests.values() for h in r['observation']['history']), 'Next model request must receive the blockage'
    if rejection:
        responses = [r for r in records if r['type'] == 'response' and r['rejection'] == rejection]
        assert responses, 'Expected rejected inference missing'
        for response in responses:
            request = requests[response['request_id']]
            late = next((r for r in records if r['type'] == 'late_response' and r['request_id'] == response['request_id']), response)
            during = [s for s in steps if request['observation']['simulation_s'] <= s['simulation_s'] <= late['simulation_s']]
            span = response['simulation_s'] - request['observation']['simulation_s']
            assert during or 0 < span <= .21, 'Physics did not advance during inference'  # Short errors fit between sampled steps.
        if rejection == 'deadline_exceeded':
            assert any(s.get('pending_reported') for s in steps), 'Rejected worker must retain its slot'
            assert all(s['applied']['mode'] == 'stop' for s in steps if s.get('pending_reported')), 'Fallback must never navigate'
            assert any(r['type'] == 'late_response' for r in records), 'Late response must be discarded and logged'
    elif end['outcome'] in ('returned_and_delivered', 'returned_empty'):
        assert set(end['delivered']) == set(end['collected'])
        assert sum(o['event'] == 'collect' for o in accepted_operations) == len(end['collected'])
        assert sum(o['event'] == 'deliver' for o in accepted_operations) == 1
    elif end['outcome'] == 'preconditions_complete':
        assert [r['ok'] for r in truth if r['type'] == 'operation'] == [False, False, False, True, True, True]
    elif end['outcome'] == 'sensors_complete':
        measured = next(r for r in records if r['type'] == 'sensor_result')
        assert measured['image_bytes'] == 320 * 240 * 4
        assert measured['lidar_finite_returns'] > 0 and measured['observed']
        assert all(math.isfinite(v) for v in measured['compass'])
        assert (directory / 'sensors.png').is_file()
    elif end['outcome'] == 'motion_complete':
        measured = next(r for r in records if r['type'] == 'motion_result')
        scale = measured.get('speed_multiplier', 1)
        stage = lambda t: min(steps, key=lambda s: abs(s['simulation_s'] - (2 + (t - 2) / scale)))
        assert stage(10)['position'][0] - stage(2)['position'][0] > .2, 'Forward axis incorrect'
        assert stage(16)['yaw'] > stage(10)['yaw'] + .08, 'Left steering not observed'
        assert stage(27)['yaw'] - stage(16)['yaw'] > .4, 'Left spin not observed'
        assert stage(27)['yaw'] - stage(38)['yaw'] > .4, 'Right spin not observed'
        assert stage(40)['speed_m_s'] < .006, 'Stop not observed'
    print(json.dumps({'run': directory.name, 'outcome': end['outcome'], 'checked_frames': len(requests),
                      'physical_operations': len(accepted_operations), 'rejection_checked': rejection}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--rejection')
    parser.add_argument('--blocked', action='store_true')
    args = parser.parse_args()
    verify(args.directory, args.rejection, args.blocked)
