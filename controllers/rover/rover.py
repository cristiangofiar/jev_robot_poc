"""Sojourner mission controller. Every Webots call stays on the physics thread."""
import json
import math
import os
import sys
from datetime import datetime
from pathlib import Path
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from controller import Robot
from mission.brain import Brain, Pending, criteria, rules
from mission.environment import load_env
from mission.logging import Log
from mission.navigation import Navigator, safety, wrap


class Drive:
    def __init__(self, robot):
        self.wheels = {side: [robot.getDevice(part + side + "Wheel") for part in ("Front", "Middle", "Back")] for side in ("Left", "Right")}
        self.arms = {part + side: robot.getDevice(part + side + "Arm") for part in ("Front", "Back") for side in ("Left", "Right")}
        for wheels in self.wheels.values():
            for motor in wheels:
                motor.setPosition(float("inf"))
                motor.setVelocity(0)
        self.command("stop", 0)

    def command(self, mode, turn):
        if mode == "spin":
            angles = {"FrontLeft": -.87, "FrontRight": .87, "BackLeft": .87, "BackRight": -.87}
            speeds = {"Left": .6 * turn, "Right": -.6 * turn}
        elif mode == "forward":
            # Same four-wheel steering geometry as the official C controller.
            angles = {"FrontLeft": .4 * turn if turn > 0 else .227 * turn,
                      "FrontRight": .227 * turn if turn > 0 else .4 * turn,
                      "BackLeft": -.227 * turn if turn > 0 else -.4 * turn,
                      "BackRight": -.4 * turn if turn > 0 else -.227 * turn}
            speeds = {"Left": -.6, "Right": -.6}
        else:
            angles = {key: 0 for key in self.arms}
            speeds = {"Left": 0, "Right": 0}
        for name, motor in self.arms.items():
            motor.setPosition(angles[name])
        for side, wheels in self.wheels.items():
            # Passive middle wheels during steering/spin, official controller behavior.
            wheels[1].setAvailableTorque(0 if mode == "spin" or (mode == "forward" and abs(turn) > .1) else 2)
            for motor in wheels:
                motor.setVelocity(speeds[side])
        return {"mode": mode, "steering": angles, "wheel_rad_s": speeds}


def run():
    load_env(ROOT / ".env")
    name = os.environ.get("MARS_BRAIN", "rules")
    deadline = float(os.environ.get("MARS_DEADLINE_S", "2"))
    max_age = float(os.environ.get("MARS_MAX_AGE_S", "3"))
    duration = float(os.environ.get("MARS_DURATION_S", "900"))
    test = os.environ.get("MARS_TEST", "mission")
    brain = Brain(name, deadline)
    directory = Path(os.environ.get("MARS_RUN_DIR", str(ROOT / "results" / (datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid4().hex[:6]))))
    log = Log(directory)
    (directory / "frames").mkdir()
    robot = Robot()
    drive = Drive(robot)
    gps, imu, compass = [robot.getDevice("mission " + device) for device in ("gps", "imu", "compass")]
    camera, lidar = robot.getDevice("front camera"), robot.getDevice("mission lidar")
    receiver, emitter = robot.getDevice("mission rx"), robot.getDevice("mission tx")
    for sensor in (gps, imu, compass, lidar, receiver):
        sensor.enable(16)
    lidar.enablePointCloud()
    camera.enable(64)
    camera.recognitionEnable(64)
    robot.getDevice("camera view").attachCamera(camera)
    navigator = Navigator()
    def send(data):
        emitter.send(json.dumps(data).encode())
    send({"type": "start", "directory": str(directory), "brain": name, "test": test})
    log.write("start", brain=name, test=test, deadline_s=deadline, maximum_age_s=max_age, duration_s=duration,
              sources=log.snapshot(ROOT), world_format="R2025a", webots_version=os.environ.get("WEBOTS_VERSION"))
    known, inspected, collected, delivered, history = {}, set(), set(), set(), []
    revision, request_id, operation_id = 0, 0, 0
    active, target, pending, dwell, operation = None, None, None, None, None
    next_request, last_log, last_ui, last_safety = 1.0, -1, -1, None
    safety_count = 0
    survey = [(1, -1.4), (2.6, -1.4), (3.1, .9), (.4, 1.4)]
    survey_index, active_since, path_length, previous_position = 0, 0, 0, None
    last_decision, last_latency, last_confidence, last_rejection, outcome = None, None, None, None, "incomplete"
    initial_position, initial_yaw = None, None
    mode, reason, distance_to_target = "stop", None, None
    try:
        while robot.step(16) != -1:
            now = robot.getTime()
            position = list(gps.getValues())
            roll, pitch, body_yaw = imu.getRollPitchYaw()
            yaw = wrap(body_yaw + math.pi)  # Official Sojourner's front is local -X.
            points = [(p.x, p.y, p.z) for p in lidar.getPointCloud()]
            ranges = [math.hypot(p[0], p[1]) for p in points]
            speed = gps.getSpeed()
            valid = all(math.isfinite(v) for v in (*position, roll, pitch, yaw, speed)) and bool(ranges) and not any(math.isnan(v) for v in ranges)
            if not valid:
                drive.command("stop", 0)
                if now > 2:
                    raise RuntimeError("Sensors did not produce valid readings")
                continue
            if initial_position is None:
                initial_position, initial_yaw = position[:], yaw
            if previous_position:
                path_length += math.dist(position[:2], previous_position[:2])
            previous_position = position[:]
            for obj in camera.getRecognitionObjects():
                sid = obj.getModel()
                if isinstance(sid, bytes):
                    sid = sid.decode()
                if sid not in ("S1", "S2", "S3") or sid in collected:
                    continue
                x, y, z = obj.getPosition()
                # ponytail: yaw-only projection for <2 degree slopes; use full IMU rotation on rough terrain.
                # Camera axes: +X optical forward, +Y left, +Z up; tilt 0.15 rad.
                forward = .38 + math.cos(.15) * x + math.sin(.15) * z
                point = [position[0] + forward * math.cos(yaw) - y * math.sin(yaw),
                         position[1] + forward * math.sin(yaw) + y * math.cos(yaw)]
                if sid not in known:
                    revision += 1
                    log.write("detected", simulation_s=now, sample=sid, sensor="camera recognition", position=point)
                known[sid] = point
            navigator.observe(position, yaw, points, list(known.values()))
            while receiver.getQueueLength():
                ack = json.loads(receiver.getString())
                receiver.nextPacket()
                if operation and ack.get("operation_id") == operation["operation_id"]:
                    log.write("operation_result", simulation_s=now, **ack)
                    if ack["ok"]:
                        if ack["event"] == "inspect":
                            inspected.add(ack["sample"])
                        elif ack["event"] == "collect":
                            collected.add(ack["sample"])
                        elif ack["event"] == "deliver":
                            delivered.update(ack["samples"])
                            outcome = "success" if len(delivered) == 3 else "partial_delivery"
                        revision += 1
                    operation, active, target, dwell = None, None, None, None
                    next_request = now + .1
            if len(delivered) == 3 or outcome == "partial_delivery":
                break
            candidates = sorted([{"id": sid, "distance_m": round(math.dist(position[:2], xy), 2), "inspected": sid in inspected}
                                 for sid, xy in known.items() if sid not in collected], key=lambda c: (c["distance_m"], c["id"]))
            observation = {"simulation_s": now, "revision": revision, "battery_pct": round(max(0, 100 - now * .045 - path_length * .2), 1),
                           "candidates": candidates, "collected": sorted(collected), "delivered": sorted(delivered),
                           "sensors_valid": valid, "obstacle": min(ranges[150:210]) < .65,
                           "history": history[-3:], "frame": None, "current_action": active}

            def apply(choice, fallback=None):
                nonlocal active, target, active_since, dwell, survey_index
                active, target, active_since, dwell = choice, None, now, None
                if choice.startswith(("INSPECT_", "COLLECT_")):
                    xy = known[choice.split("_")[1]]
                    # Select a reachable standoff around the observed marker, without hidden geometry.
                    goals = [(xy[0] + .55 * math.cos(a * math.pi / 8), xy[1] + .55 * math.sin(a * math.pi / 8)) for a in range(16)]
                    routes = [(g, navigator.plan(position, g)) for g in goals]
                    routes = [(g, p) for g, p in routes if p]
                    if routes:
                        target = min(routes, key=lambda pair: len(pair[1]))[0]
                    else:
                        active = "WAIT"
                        fallback = "no_route"
                elif choice == "RETURN":
                    target = (0, 0)
                elif choice == "EXPLORE":
                    target = survey[survey_index % len(survey)]
                history.append({"requested": last_decision, "applied": active, "fallback": fallback})
                del history[:-3]
                log.write("action", simulation_s=now, request_id=request_id, requested=last_decision, applied=active, fallback=fallback, target=target)

            if pending:
                result = pending.poll(now, revision, deadline, max_age)
                if result:
                    last_decision, last_latency = result["decision"].get("choice"), result["latency_ms"]
                    last_confidence = result["decision"].get("confidence")
                    rejection = result["rejection"]
                    if last_decision is not None and last_decision not in criteria(observation):
                        rejection = "no_longer_available"
                    last_rejection = rejection
                    if rejection == "deadline_exceeded":
                        last_latency = None  # Actual inference duration is not known yet.
                    log.write("response", simulation_s=now, request_id=pending.request_id, observation_s=pending.observation["simulation_s"],
                              observation_revision=pending.observation["revision"], current_revision=revision, **{**result, "rejection": rejection})
                    apply(rules(observation) if rejection or last_decision is None else last_decision, rejection)
                # A timed-out thread continues occupying this slot until it exits.
                if pending.reported and not pending.thread.is_alive():
                    if not pending.queue.empty():
                        late, finished = pending.queue.get_nowait()
                        last_latency = (finished - pending.started) * 1000
                        log.write("late_response", simulation_s=now, request_id=pending.request_id, decision=late,
                                  rejection="already_rejected", latency_ms=(finished - pending.started) * 1000)
                    pending = None
            if active in ("WAIT", "EXPLORE") and now - active_since > (3 if active == "WAIT" else 12):
                active, target = None, None
                next_request = now
            if active is None and pending is None and now >= next_request and test == "mission" and abs(now / .064 - round(now / .064)) < 1e-6:
                request_id += 1
                frame = f"frames/request_{request_id:04d}.png"
                if camera.saveImage(str(directory / frame), 100) != 0:
                    raise RuntimeError("Camera frame save failed")
                # Request is submitted only on a fresh camera tick (timestamps logged explicitly).
                observation["frame"] = {"path": frame, "simulation_s": math.floor((now + 1e-8) / .064) * .064, "width": 320, "height": 240}
                log.write("request", request_id=request_id, observation=observation,
                          payload=brain.payload(observation) if name in ("laya", "kev", "jev") else None)
                pending = Pending(brain, observation, request_id)
            if test == "motion":
                mode, turn = ("stop", 0) if now < 2 else ("forward", 0) if now < 12 else ("forward", 1) if now < 24 else ("spin", 1) if now < 38 else ("stop", 0)
                if now > 40:
                    outcome = "motion_complete"
                    log.write("motion_result", displacement_m=math.dist(position[:2], initial_position[:2]), yaw_change_rad=wrap(yaw - initial_yaw), speed_m_s=speed)
                    break
            elif test == "sensors":
                mode, turn = "stop", 0
                if now > 2:
                    camera.saveImage(str(directory / "sensors.png"), 100)
                    log.write("sensor_result", position=position, roll=roll, pitch=pitch, yaw=yaw, compass=compass.getValues(),
                              lidar_points=[p for p in points if all(math.isfinite(v) for v in p)][::8], obstacle_cells=sorted(navigator.obstacles), lidar_finite_returns=sum(math.isfinite(v) for v in ranges), image_bytes=len(camera.getImage()), observed=sorted(known))
                    outcome = "sensors_complete"
                    break
            elif target is not None and not operation:
                mode, turn = navigator.command(position, yaw, target, now)
                distance_to_target = math.dist(position[:2], target)
                if mode == "arrived":
                    mode = "stop"
                    if active == "EXPLORE":
                        survey_index += 1
                        active, target = None, None
                    else:
                        if dwell is None:
                            dwell = now
                        if now - dwell > 1.5:
                            operation_id += 1
                            event = "deliver" if active == "RETURN" else "inspect" if active.startswith("INSPECT_") else "collect"
                            operation = {"type": "operation", "event": event, "sample": active.split("_")[-1] if event != "deliver" else None,
                                         "operation_id": operation_id, "observed": sorted(known)}
                            send(operation)
                            log.write("operation_request", simulation_s=now, **operation)
                elif mode == "no_route":
                    mode = "stop"
                    if now - active_since > 8:
                        log.write("fallback", simulation_s=now, reason="no_route", active=active, goal=target, obstacle_cells=sorted(navigator.obstacles))
                        active, target = None, None
                        next_request = now + 1
            else:
                mode, turn = "stop", 0
            reason = safety(position, roll, pitch, ranges, mode)
            if reason != last_safety:
                if reason is not None:
                    safety_count += 1
                log.write("safety", simulation_s=now, reason=reason, requested_motion=mode)
                last_safety = reason
            commands = drive.command("stop" if reason else mode, turn)
            if now - last_log >= .2:
                log.write("step", simulation_s=now, position=position, yaw=yaw, roll=roll, pitch=pitch, speed_m_s=speed,
                          lidar_front_m=min(4, min(ranges[150:210])), lidar_min_m=min(4, min(ranges)),
                          active=active, requested_motion=mode, applied=commands, safety=reason,
                          pending_request=pending.request_id if pending else None, pending_reported=pending.reported if pending else None,
                          collected=sorted(collected), delivered=sorted(delivered), observed=sorted(known), path_length_m=path_length)
                last_log = now
            if now - last_ui >= .25:
                send({"type": "status", "time": round(now, 1), "active": active, "motion": commands["mode"], "decision": last_decision,
                      "latency_ms": last_latency, "collected": sorted(collected), "delivered": sorted(delivered),
                      "safety": reason, "safety_count": safety_count, "confidence": last_confidence, "fallback": last_rejection,
                      "phase": "dwell" if dwell else "awaiting decision" if active is None else "travel" if target else "wait"})
                last_ui = now
            if now >= duration:
                outcome = "time_limit"
                break
        drive.command("stop", 0)
        flushed = robot.step(0) != -1
        log.write("end", outcome=outcome, simulation_s=robot.getTime(), collected=sorted(collected), delivered=sorted(delivered),
                  path_length_m=path_length, stop_flushed=flushed)
        send({"type": "end", "outcome": outcome})
        robot.step(16)
        print(f"Mars mission: {outcome}; logs: {directory}", flush=True)
    except Exception as exc:
        drive.command("stop", 0)
        robot.step(0)
        log.write("end", outcome="error", error_type=type(exc).__name__, simulation_s=robot.getTime())
        send({"type": "end", "outcome": "error"})
        robot.step(16)
        raise
    finally:
        log.file.close()


if __name__ == "__main__":
    run()
