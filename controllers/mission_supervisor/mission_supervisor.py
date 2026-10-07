"""Physical arrival/dwell validation and visual HUD. Hidden scene data stays here."""
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from controller import Supervisor
from mission.logging import Log


def run():
    robot = Supervisor()
    receiver, emitter = robot.getDevice("mission rx"), robot.getDevice("mission tx")
    receiver.enable(16)
    rover = robot.getFromDef("ROVER")
    samples = {sid: robot.getFromDef(sid) for sid in ("S1", "S2", "S3")}
    positions = {sid: node.getPosition() for sid, node in samples.items()}
    collected, inspected, operations = set(), set(), {}
    stationary_since, log, directory, last_log, ending = None, None, None, -1, None
    status = {}
    while robot.step(16) != -1:
        now = robot.getTime()
        position, velocity = rover.getPosition(), rover.getVelocity()
        if math.hypot(*velocity[:2]) < .006:
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
            elif kind == "status":
                status = data
                latency = "-" if data["latency_ms"] is None else f"{data['latency_ms']:.0f} ms"
                confidence = "-" if data.get("confidence") is None else f"{data['confidence']:.3f}"
                text = ["MARS | SOJOURNER | 3 SAMPLE MISSION", f"Goal: {data['active'] or 'awaiting decision'} | local: {data['motion']}",
                        f"Collected: {','.join(data['collected']) or '0'} / 3 | Delivered: {len(data['delivered'])} / 3",
                        f"Model: {data['decision'] or '-'} | latency: {latency} | confidence: {confidence}",
                        f"Safety: {data['safety'] or 'clear'} | interventions: {data.get('safety_count', 0)} | {data['time']} s",
                        f"Phase: {data.get('phase', 'running')} | fallback: {data.get('fallback') or '-'}"]
                for i, line in enumerate(text):
                    robot.setLabel(i, line, .30, .025 + i * .040, .03, 0xD9F3FF, .08, "Arial")
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
                    elif sid in positions and sid in data["observed"] and sid not in collected:
                        distance = math.dist(position[:2], positions[sid][:2])
                        ack["distance_m"] = distance
                        if distance <= .68 and stopped and (event == "inspect" or (event == "collect" and sid in inspected)):
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
                    log.write("end", simulation_s=now, outcome=data["outcome"], collected=sorted(collected))
                    log.file.close()
                    log = None
                robot.setLabel(6, "MISSION: " + data["outcome"].upper(), .30, .285, .04, 0x80FF80, 0, "Arial")
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
