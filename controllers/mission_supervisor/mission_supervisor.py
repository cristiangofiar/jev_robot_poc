"""Physical arrival/dwell validation and visual HUD. Hidden scene data stays here."""
import json
import math
import os
import sys
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from controller import Supervisor
from mission.logging import Log
from mission.navigation import wrap


def show_actions(robot, actions, data):
    """Update one execution in place; new decisions push the oldest row out."""
    for entry in actions:
        if entry["started_s"] == data["started_s"]:
            entry.update(data)
            break
    else:
        actions.append(data.copy())
    for i, entry in enumerate(actions):
        color = {"blocked": 0xFFB86C, "interrupted": 0xFFB86C, "failed": 0xFF8080,
                 "completed": 0x80FF80}.get(entry["result"], 0xD9F3FF)
        if entry.get("fallback"):
            color = 0xFF8080
        reason = entry.get("safety") or entry.get("fallback") or (entry.get("reason") if entry["result"] == "failed" else None)
        text = [f"#{entry['request_id']} {entry['started_s']:.1f}s {entry.get('requested') or '(no decision)'}",
                f"-> {entry['applied']} | {entry['result']}" + (f": {reason}" if reason else "")]
        for j, line in enumerate(text):
            robot.setLabel(10 + 2*i + j, line, .015, .55 + i * .07 + j * .03, .04, color, 0, "Arial")


def run():
    robot = Supervisor()
    receiver, emitter = robot.getDevice("mission rx"), robot.getDevice("mission tx")
    receiver.enable(16)
    rover = robot.getFromDef("ROVER")
    samples = {}
    children = robot.getRoot().getField("children")
    for index in range(children.getCount()):
        node = children.getMFNode(index)
        model = node.getField("model") if node.getTypeName() == "Solid" else None
        if model:
            sid = model.getSFString()
            if sid.startswith("S") and sid[1:].isdigit():
                samples[sid] = node
    positions = {sid: node.getPosition() for sid, node in samples.items()}
    collected, inspected, delivered, operations = set(), set(), set(), {}
    stationary_since, log, directory, last_log, ending = None, None, None, -1, None
    status = {}
    actions = deque(maxlen=6)
    while robot.step(16) != -1:
        now = robot.getTime()
        position, velocity = rover.getPosition(), rover.getVelocity()
        if math.hypot(*velocity[:2]) < .006 and abs(velocity[5]) < .04:
            if stationary_since is None:
                stationary_since = now
        else:
            stationary_since = None
        while receiver.getQueueLength():
            data = json.loads(receiver.getString())
            receiver.nextPacket()
            kind = data["type"]
            if kind == "start":
                directory = Path(data["directory"])
                log = Log(directory, "ground_truth.jsonl")
                log.write("start", samples=positions, collection_radius_m=.68, required_stop_s=1,
                          brain=data["brain"], test=data["test"])
                robot.setLabel(8, "DECISIONS | " + data["brain"].upper() + " | latest 6", .015, .49, .04, 0xD9F3FF, 0, "Arial")
            elif kind == "action":
                show_actions(robot, actions, data)
            elif kind == "status":
                status = data
                latency = "-" if data["latency_ms"] is None else f"{data['latency_ms']:.0f} ms"
                confidence = "-" if data.get("confidence") is None else f"{data['confidence']:.3f}"
                wheels = data["wheel_rad_s"]
                last = data.get("last_action") or {}
                if last and actions:
                    show_actions(robot, actions, {**last, "request_id": actions[-1]["request_id"]})
                protection = data.get("last_safety_intervention")
                safety_text = data["safety"] or "inactive"
                if protection:
                    safety_text += f" | last: {protection['reason']} at {protection['simulation_s']:.1f}s"
                text = ["MARS | SOJOURNER | ITERATIVE MOVEMENT", f"Action: {data['active'] or 'awaiting decision'} | motors: {data['motion']} L={wheels['Left']:.1f} R={wheels['Right']:.1f} rad/s",
                        f"Observed: {len(data['observed'])} | Collected: {len(data['collected'])} | Delivered: {len(data['delivered'])} | Track coverage: {data['coverage_pct']}%",
                        f"Model: {data['decision'] or '-'} | latency: {latency} | confidence: {confidence}",
                        f"Safety: {safety_text} | count: {data.get('safety_count', 0)}",
                        f"Last: {last.get('requested') or '-'} -> {last.get('applied', '-')} | {last.get('result', '-')} | delta: {last.get('displacement_m', 0):.4f} m / {last.get('yaw_change_deg', 0):.2f} deg",
                        f"Phase: {data.get('phase', 'running')} | fallback: {data.get('fallback') or '-'} | {data['time']} s"]
                for i, line in enumerate(text):
                    robot.setLabel(i, line, .37, .025 + i * .050, .05, 0xD9F3FF, .08, "Arial")
            elif kind == "operation":
                oid = data["operation_id"]
                if oid in operations:
                    ack = operations[oid]
                else:
                    event, sid = data["event"], data.get("sample")
                    stopped = stationary_since is not None and now - stationary_since >= 1
                    ack = {"operation_id": oid, "event": event, "sample": sid, "ok": False, "reason": "physical_precondition"}
                    if event == "deliver":
                        if math.hypot(*position[:2]) <= .3 and stopped:
                            ack.update(ok=True, reason=None, samples=sorted(collected))
                            delivered.update(collected)
                    elif sid in positions and sid in data["observed"] and sid not in collected:
                        distance = math.dist(position[:2], positions[sid][:2])
                        ack["distance_m"] = distance
                        orientation = rover.getOrientation()
                        yaw = math.atan2(-orientation[3], -orientation[0])
                        bearing = math.degrees(wrap(math.atan2(positions[sid][1] - position[1], positions[sid][0] - position[0]) - yaw))
                        ack["bearing_deg"] = bearing
                        ack["visible"] = sid in data.get("visible", [])
                        inspection_view = ack["visible"] and abs(bearing) <= 20
                        if distance <= .68 and stopped and ((event == "inspect" and inspection_view) or (event == "collect" and sid in inspected)):
                            ack.update(ok=True, reason=None)
                            if event == "inspect":
                                inspected.add(sid)
                            else:
                                collected.add(sid)
                                samples[sid].getField("translation").setSFVec3f([*positions[sid][:2], -1])
                    operations[oid] = ack
                    if log:
                        log.write("operation", simulation_s=now, physical_position=position, stationary_since=stationary_since, **ack)
                emitter.send(json.dumps(ack).encode())
            elif kind == "end":
                ending = now
                if log:
                    log.write("end", simulation_s=now, outcome=data["outcome"], collected=sorted(collected),
                              delivered=sorted(delivered), total_samples=len(samples),
                              undiscovered=sorted(set(samples) - set(data.get("observed", []))),
                              uncollected=sorted(set(samples) - collected), all_delivered=set(samples) <= delivered)
                    log.file.close()
                    log = None
                robot.setLabel(7, "MISSION: " + data["outcome"].upper(), .37, .375, .05, 0x80FF80, 0, "Arial")
                if directory:
                    robot.exportImage(str(directory / "world_final.png"), 100)
        if log and now - last_log >= .2:
            contacts = rover.getContactPoints(True)
            # Floor/wheel contacts are expected: only contacts above the terrain count as hazards.
            obstacle_contacts = [c.point for c in contacts if c.point[2] > .09]
            log.write("trajectory", simulation_s=now, position=position, velocity=velocity,
                      obstacle_contacts=obstacle_contacts, collected=sorted(collected))
            last_log = now
        if ending is not None and now - ending > .08:
            if os.environ.get("MARS_BATCH") == "1":
                robot.simulationQuit(0 if status.get("safety") != "invalid_sensors" else 1)
                break


if __name__ == "__main__":
    run()
