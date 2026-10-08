"""Sojourner mission controller. Every Webots call stays on the physics thread."""
import json
import math
import os
import sys
from datetime import datetime
from pathlib import Path
from uuid import uuid4
from copy import deepcopy

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from controller import Robot
from mission.brain import Brain, Pending, criteria, rules
from mission.environment import load_env
from mission.logging import ActionExecution, execution_history, start_run_log
from mission.navigation import BASE, BOUNDS, Coverage, lidar_summary, relative, safety, wrap


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
            speeds = {"Left": 1.8 * turn, "Right": -1.8 * turn}
        elif mode in ("forward", "reverse"):
            # Same four-wheel steering geometry as the official C controller.
            angles = {"FrontLeft": .4 * turn if turn > 0 else .227 * turn,
                      "FrontRight": .227 * turn if turn > 0 else .4 * turn,
                      "BackLeft": -.227 * turn if turn > 0 else -.4 * turn,
                      "BackRight": -.4 * turn if turn > 0 else -.227 * turn}
            speeds = {"Left": 1.8 if mode == "reverse" else -1.8, "Right": 1.8 if mode == "reverse" else -1.8}
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
    duration = float(os.environ.get("MARS_DURATION_S", "600"))
    interval = float(os.environ.get("MARS_ACTION_S", ".8"))
    explore_budget = float(os.environ.get("MARS_EXPLORE_S", "240"))
    coverage_target = float(os.environ.get("MARS_COVERAGE_PCT", "65"))
    if not all(math.isfinite(value) and value > 0 for value in (deadline, duration, interval, explore_budget)) or interval > 1 or not 0 < coverage_target <= 100:
        raise ValueError("Invalid mission limits")
    test = os.environ.get("MARS_TEST", "mission")
    brain = Brain(name, deadline)
    directory = Path(os.environ.get("MARS_RUN_DIR", str(ROOT / "results" / (datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid4().hex[:6]))))
    log = start_run_log(directory, brain=name)
    directory = log.directory
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
    coverage = Coverage()
    baseline = None
    if name in ("rules", "slow"):
        from mission.baseline import Navigator
        baseline = Navigator()
    def send(data):
        emitter.send(json.dumps(data).encode())
    send({"type": "start", "directory": str(directory), "brain": name, "test": test})
    log.write("start", brain=name, test=test, deadline_s=deadline, duration_s=duration,
              action_duration_s=interval, explore_budget_s=explore_budget, coverage_target_pct=coverage_target,
              sources=log.snapshot(ROOT), world_format="R2025a", webots_version=os.environ.get("WEBOTS_VERSION"))
    known, inspected, collected, delivered, history = {}, set(), set(), set(), []
    revision, request_id, operation_id = 0, 0, 0
    active, pending, operation, action_until = None, None, None, 0
    interaction = None
    returning, goal_id, stopped_since = False, None, None
    last_log, last_ui, last_safety, safety_count = -1, -1, None, 0
    path_length, previous_position, recent = 0, None, []
    last_decision, last_latency, last_confidence, last_rejection, outcome = None, None, None, None, "incomplete"
    initial_position, initial_yaw = None, None
    commands = drive.command("stop", 0)
    observation = None
    action_source = None
    last_safety_intervention = None
    precondition_stage = 0
    precondition_results = []
    execution = None

    def finish_action(reason, protection=None, operation_ok=None):
        nonlocal execution
        if execution is not None:
            execution.finish(reason, safety=protection, operation_ok=operation_ok)
            log.write("action_result", simulation_s=execution.time, request_id=request_id, source=action_source, **execution.entry)
            send({"type": "action", "request_id": request_id, **execution.entry})
            execution = None

    try:
        while robot.step(16) != -1:
            now = robot.getTime()
            position = list(gps.getValues())
            roll, pitch, body_yaw = imu.getRollPitchYaw()
            yaw = wrap(body_yaw + math.pi)
            points = [(p.x, p.y, p.z) for p in lidar.getPointCloud()]
            speed = gps.getSpeed()
            valid = all(math.isfinite(v) for v in (*position, roll, pitch, yaw, speed)) and bool(points) and not any(math.isnan(math.hypot(p[0], p[1])) for p in points)
            if not valid:
                commands = drive.command("stop", 0)
                if execution:
                    execution.command(commands)
                    finish_action("invalid_sensors", "invalid_sensors")
                if now > 2:
                    log.write("safety", simulation_s=now, reason="invalid_sensors", requested_motion="stop")
                    raise RuntimeError("Sensors did not produce valid readings")
                continue
            sectors = lidar_summary(points)
            if execution:
                execution.observe(now, position, yaw)
            if initial_position is None:
                initial_position, initial_yaw = position[:], yaw
            if previous_position:
                path_length += math.dist(position[:2], previous_position[:2])
                yaw_rate = wrap(yaw - previous_yaw) / .016
            else:
                yaw_rate = 0
            previous_position, previous_yaw = position[:], yaw
            if speed < .006 and abs(yaw_rate) < .04:
                if stopped_since is None:
                    stopped_since = now
            else:
                stopped_since = None
            stopped_s = 0 if stopped_since is None else now - stopped_since
            coverage.observe(position)
            visible = set()
            for obj in camera.getRecognitionObjects():
                sid = obj.getModel()
                if isinstance(sid, bytes):
                    sid = sid.decode()
                if not (sid.startswith("S") and sid[1:].isdigit()) or sid in collected:
                    continue
                x, y, z = obj.getPosition()
                # ponytail: yaw projection for this gentle terrain; full IMU transform for rough slopes.
                forward = .38 + math.cos(.15) * x + math.sin(.15) * z
                point = [position[0] + forward * math.cos(yaw) - y * math.sin(yaw),
                         position[1] + forward * math.sin(yaw) + y * math.cos(yaw)]
                visible.add(sid)
                if sid not in known:
                    revision += 1
                    log.write("detected", simulation_s=now, sample=sid, sensor="camera recognition", position=point)
                known[sid] = {"position": point, "last_seen_s": now}
            while receiver.getQueueLength():
                ack = json.loads(receiver.getString())
                receiver.nextPacket()
                if operation and ack.get("operation_id") == operation["operation_id"]:
                    log.write("operation_result", simulation_s=now, **ack)
                    if test == "preconditions":
                        precondition_results.append(ack["ok"])
                    if ack["ok"]:
                        if ack["event"] == "inspect":
                            inspected.add(ack["sample"])
                        elif ack["event"] == "collect":
                            collected.add(ack["sample"])
                            goal_id = None
                        elif ack["event"] == "deliver":
                            delivered.update(ack["samples"])
                            outcome = "returned_and_delivered" if delivered else "returned_empty"
                        revision += 1
                    if active:
                        log.write("action_end", simulation_s=now, accepted=active, reason="physical_validation", ok=ack["ok"])
                        finish_action("physical_validation", operation_ok=ack["ok"])
                    operation = None
                    active = None
            if outcome in ("returned_and_delivered", "returned_empty"):
                if test == "preconditions":
                    if precondition_results != [False, False, False, True, True, True]:
                        raise RuntimeError("Unexpected physical precondition results")
                    outcome = "preconditions_complete"
                break
            candidates = []
            for sid, memory in known.items():
                if sid in collected:
                    continue
                candidate = {"id": sid, **relative(position, yaw, memory["position"]), "visible": sid in visible,
                             "inspected": sid in inspected, "last_seen_ago_s": round(now - memory["last_seen_s"], 1)}
                arrived = candidate["distance_m"] <= .63
                aligned = candidate["visible"] and abs(candidate["bearing_deg"]) <= 18
                candidate["inspect_ready"] = arrived and aligned and sid not in inspected
                candidate["collect_ready"] = arrived and sid in inspected
                candidates.append(candidate)
            candidates.sort(key=lambda c: (c["distance_m"], c["id"]))
            base = relative(position, yaw, BASE)
            explore = coverage.state(position, yaw)
            battery = max(0, 100 - now * .045 - path_length * .2)
            # Reserve uses only public base geometry and conservative nominal drive speed.
            return_reserve_s = 2 * base["distance_m"] / .078 + 35
            return_reason = ("battery_reserve" if battery <= 20 else "time_reserve" if duration - now <= return_reserve_s else
                             "exploration_budget" if now >= explore_budget else "coverage_threshold" if explore["visited_pct"] >= coverage_target else None)
            if returning:
                goal = {"kind": "base", **base}
            elif candidates:
                if goal_id not in {c["id"] for c in candidates}:
                    goal_id = candidates[0]["id"]
                goal = {"kind": "sample", **next(c for c in candidates if c["id"] == goal_id)}
            else:
                goal = {"kind": "exploration"}
            if now - last_log >= .2:
                recent.append((now, position[:2], yaw))
                recent = [entry for entry in recent if now - entry[0] <= 8]
            advance = math.dist(position[:2], recent[0][1]) if recent else 0
            turned = abs(wrap(yaw - recent[0][2])) if recent else 0
            repeated = len(history) >= 4 and len({h["requested"] for h in history[-4:]}) == 1
            observation = {"simulation_s": now, "revision": revision, "pose": [*position[:2], yaw],
                           "battery_pct": round(battery, 1), "time_remaining_s": round(max(0, duration - now), 1),
                           "explore_remaining_s": round(max(0, explore_budget - now), 1), "return_due": return_reason is not None,
                           "return_reason": return_reason, "goal": goal, "base": base, "candidates": candidates,
                           "collected": sorted(collected), "delivered": sorted(delivered), "sensors_valid": valid, "lidar_m": sectors,
                           "motion": {"applied_motor": commands["mode"], "wheel_rad_s": commands["wheel_rad_s"].copy(), "speed_m_s": round(speed, 4), "yaw_deg": round(math.degrees(yaw), 1),
                                      "roll_deg": round(math.degrees(roll), 1), "pitch_deg": round(math.degrees(pitch), 1)},
                           "exploration": explore, "progress": {"window_s": 8, "displacement_m": round(advance, 3),
                                      "turn_deg": round(math.degrees(turned), 1), "no_progress": advance < .01 and turned < .05,
                                      "repeated_action": repeated}, "history": execution_history(history), "safety": last_safety, "last_safety_intervention": last_safety_intervention, "fallback": last_rejection,
                           "action_duration_s": interval, "stopped_s": round(stopped_s, 2), "frame": None,
                           "boundary_clearance_m": {"x_min": round(position[0]-BOUNDS[0], 2), "x_max": round(BOUNDS[1]-position[0], 2),
                                                     "y_min": round(position[1]-BOUNDS[2], 2), "y_max": round(BOUNDS[3]-position[1], 2)}}
            if baseline is not None:
                baseline.observe(position, yaw, points, [memory["position"] for memory in known.values()])
                if (active is None or now >= action_until) and pending is None and not operation and now >= 1 and abs(now / .064 - round(now / .064)) < 1e-6:
                    if goal["kind"] == "base":
                        rules_goal = BASE
                    elif goal["kind"] == "sample":
                        xy = known[goal_id]["position"]
                        # Standoff is exclusive to the independent deterministic comparison.
                        goals = [(xy[0] + .55 * math.cos(a * math.pi / 8), xy[1] + .55 * math.sin(a * math.pi / 8)) for a in range(16)]
                        routes = [(g, baseline.plan(position, g)) for g in goals]
                        routes = [(g, route) for g, route in routes if route]
                        rules_goal = min(routes, key=lambda pair: len(pair[1]))[0] if routes else None
                    else:
                        rules_goal = coverage.frontier(position)
                    if rules_goal is not None:
                        rules_mode, rules_turn = baseline.command(position, yaw, rules_goal, now)
                        observation["rules_navigation"] = {"mode": rules_mode, "turn": rules_turn}
                    else:
                        observation["rules_navigation"] = {"mode": "no_route", "turn": 0}

            def apply(choice, fallback=None):
                nonlocal active, action_until, returning, revision, interaction, action_source, execution
                finish_action("superseded")
                active, action_until, interaction = choice, now + interval, None
                if choice == "RETURN_TO_BASE":
                    returning = True
                    revision += 1
                elif choice.startswith(("INSPECT_", "COLLECT_")):
                    event, sid = choice.split("_", 1)
                    interaction = {"event": event.lower(), "sample": sid}
                    action_until = now + 3  # Brake + short physical dwell, never approach.
                action_source = "fallback" if fallback else "deterministic" if name in ("rules", "slow") else "model"
                execution = ActionExecution(last_decision, choice, fallback, now, position, yaw)
                history.append(execution.entry)
                del history[:-4]
                log.write("action", simulation_s=now, request_id=request_id, requested=last_decision, accepted=active,
                          fallback=fallback, source=action_source,
                          duration_s=action_until-now, goal=goal)
                send({"type": "action", "request_id": request_id, **execution.entry})

            if active is not None and now >= action_until:
                commands = drive.command("stop", 0)
                if execution:
                    execution.command(commands)
                log.write("action_end", simulation_s=now, accepted=active, reason="interval_elapsed")
                finish_action("stop_not_achieved" if interaction else "interval_elapsed")
                active = None
                if interaction:
                    log.write("operation_refused", simulation_s=now, reason="stop_not_achieved", **interaction)
                    interaction = None
            if pending:
                result = pending.poll(observation, deadline)
                if result:
                    last_decision, last_latency = result["decision"].get("choice"), result["latency_ms"]
                    last_confidence = result["decision"].get("confidence")
                    rejection = result["rejection"]
                    if last_decision is not None and last_decision not in criteria(observation):
                        rejection = "no_longer_available"
                    if last_decision is None and rejection is None:
                        rejection = "invalid_response"
                    last_rejection = rejection
                    if rejection == "deadline_exceeded":
                        last_latency = None
                    log.write("response", simulation_s=now, request_id=pending.request_id, observation_s=pending.observation["simulation_s"],
                              observation_revision=pending.observation["revision"], current_revision=revision, **{**result, "rejection": rejection})
                    apply("STOP" if rejection else last_decision, rejection)
                if pending.reported and not pending.thread.is_alive():
                    if not pending.queue.empty():
                        late, finished = pending.queue.get_nowait()
                        last_latency = (finished - pending.started) * 1000
                        log.write("late_response", simulation_s=now, request_id=pending.request_id, decision=late,
                                  rejection="already_rejected", latency_ms=last_latency)
                    pending = None
            if interaction and stopped_s >= 1.2:
                sid = interaction["sample"]
                candidate = next((c for c in candidates if c["id"] == sid), None)
                ready_key = "inspect_ready" if interaction["event"] == "inspect" else "collect_ready"
                if candidate and candidate[ready_key]:
                    operation_id += 1
                    operation = {"type": "operation", **interaction, "operation_id": operation_id,
                                 "observed": sorted(known), "visible": sorted(visible)}
                    send(operation)
                    log.write("operation_request", simulation_s=now, source=action_source, **operation)
                    interaction = None
                else:
                    log.write("operation_refused", simulation_s=now, reason="arrival_or_view_lost", **interaction)
                    finish_action("arrival_or_view_lost", operation_ok=False)
                    apply("STOP", "operation_precondition_lost")
            # Return delivery is local bookkeeping after model-controlled arrival, never navigation.
            if returning and base["distance_m"] <= .27 and stopped_s >= 1.2 and not operation:
                operation_id += 1
                operation = {"type": "operation", "event": "deliver", "sample": None, "operation_id": operation_id,
                             "observed": sorted(known), "visible": sorted(visible)}
                send(operation)
                log.write("operation_request", simulation_s=now, **operation)
            if active is None and pending is None and not operation and now >= 1 and test == "mission" and abs(now / .064 - round(now / .064)) < 1e-6:
                request_id += 1
                frame = f"frames/request_{request_id:04d}.png"
                if camera.saveImage(str(directory / frame), 100) != 0:
                    raise RuntimeError("Camera frame save failed")
                observation["frame"] = {"path": frame, "simulation_s": now, "width": 320, "height": 240}
                # An interval may have ended since this tick's sensor observation was built.
                # Keep both the saved request and its worker snapshot independent of future updates.
                observation["history"] = execution_history(history)
                observation["motion"].update(applied_motor=commands["mode"], wheel_rad_s=commands["wheel_rad_s"].copy())
                log.write("request", request_id=request_id, observation=observation,
                          payload=brain.payload(observation) if name in ("laya", "kev", "jev") else None)
                pending = Pending(brain, observation, request_id)
            if test == "preconditions":
                mode, turn = "stop", 0
                if known and not operation and precondition_stage < 6 and now >= (.256, 1.5, 2, 2.5, 3, 3.5)[precondition_stage]:
                    sid = min(known, key=lambda key: math.dist(position[:2], known[key]["position"]))
                    event = ("inspect", "collect", "inspect", "inspect", "collect", "deliver")[precondition_stage]
                    operation_id += 1
                    operation = {"type": "operation", "event": event, "sample": sid if event != "deliver" else None,
                                 "operation_id": operation_id, "observed": sorted(known),
                                 "visible": [] if precondition_stage == 2 else sorted(visible)}
                    send(operation)
                    log.write("operation_request", simulation_s=now, source="physical_fixture", **operation)
                    precondition_stage += 1
            elif test == "motion":
                # Preserve the fixture's travel and turns at triple motor speed.
                motion_s = 2 + 3 * (now - 2)
                mode, turn = ("stop", 0) if motion_s < 2 else ("forward", 0) if motion_s < 10 else ("forward", 1) if motion_s < 16 else ("spin", 1) if motion_s < 27 else ("spin", -1) if motion_s < 38 else ("stop", 0)
                if motion_s > 44:  # Keep two physical seconds to verify braking.
                    outcome = "motion_complete"
                    log.write("motion_result", displacement_m=math.dist(position[:2], initial_position[:2]), yaw_change_rad=wrap(yaw - initial_yaw), speed_m_s=speed, speed_multiplier=3)
                    break
            elif test == "sensors":
                mode, turn = "stop", 0
                if now > 2:
                    camera.saveImage(str(directory / "sensors.png"), 100)
                    log.write("sensor_result", position=position, roll=roll, pitch=pitch, yaw=yaw, compass=compass.getValues(),
                              lidar_points=[p for p in points if all(math.isfinite(v) for v in p)][::8], lidar_m=sectors,
                              lidar_finite_returns=sum(all(math.isfinite(v) for v in p) for p in points), image_bytes=len(camera.getImage()), observed=sorted(known))
                    outcome = "sensors_complete"
                    break
            else:
                mode = "stop" if operation else "forward" if active == "FORWARD" else "reverse" if active == "REVERSE" else "spin" if active in ("TURN_LEFT", "TURN_RIGHT") else "stop"
                turn = 1 if active == "TURN_LEFT" else -1 if active == "TURN_RIGHT" else 0
            reason = safety(position, roll, pitch, sectors, mode, yaw, valid)
            if reason != last_safety:
                if reason is not None:
                    safety_count += 1
                    last_safety_intervention = {"reason": reason, "simulation_s": now, "requested": active}
                log.write("safety", simulation_s=now, reason=reason, requested_motion=mode, action=active)
                last_safety = reason
            commands = drive.command("stop" if reason else mode, turn)
            if execution:
                execution.command(commands)
            if reason and active is not None and mode != "stop":
                log.write("action_end", simulation_s=now, accepted=active, reason=reason)
                finish_action(reason, protection=reason)
                active = None  # An interrupted movement never resumes without a new decision.
            if now - last_log >= .2:
                log.write("step", simulation_s=now, position=position, yaw=yaw, roll=roll, pitch=pitch, speed_m_s=speed,
                          lidar_front_m=sectors["front"], lidar_min_m=min(sectors.values()), active=active,
                          requested_motion=mode, applied=commands, safety=reason, action_source=action_source,
                          pending_request=pending.request_id if pending else None, pending_reported=pending.reported if pending else None,
                          collected=sorted(collected), delivered=sorted(delivered), observed=sorted(known), visible=sorted(visible),
                          path_length_m=path_length, visited_pct=explore["visited_pct"], no_progress=observation["progress"]["no_progress"])
                last_log = now
            if now - last_ui >= .25:
                send({"type": "status", "time": round(now, 1), "active": active, "motion": commands["mode"], "decision": last_decision,
                      "latency_ms": last_latency, "collected": sorted(collected), "delivered": sorted(delivered), "observed": sorted(known),
                      "safety": reason, "last_safety_intervention": last_safety_intervention,
                      "last_action": deepcopy(history[-1]) if history else None, "wheel_rad_s": commands["wheel_rad_s"],
                      "safety_count": safety_count, "confidence": last_confidence, "fallback": last_rejection,
                      "coverage_pct": explore["visited_pct"], "phase": "return" if returning else "operation" if operation else "inference" if pending else "exploration"})
                last_ui = now
            if now >= duration:
                outcome = "time_limit"
                break
        commands = drive.command("stop", 0)
        if execution:
            execution.command(commands)
        finish_action("mission_ended")
        flushed = robot.step(0) != -1
        # Drain a completed/rejected worker for traceability without blocking physics.
        if pending and not pending.reported:
            log.write("response", simulation_s=robot.getTime(), request_id=pending.request_id,
                      decision={"choice": None}, rejection="mission_ended", latency_ms=None)
        if pending and not pending.thread.is_alive() and not pending.queue.empty():
            late, finished = pending.queue.get_nowait()
            log.write("late_response", simulation_s=robot.getTime(), request_id=pending.request_id, decision=late,
                      rejection="mission_ended", latency_ms=(finished - pending.started) * 1000)
        log.write("end", outcome=outcome, simulation_s=robot.getTime(), observed=sorted(known), collected=sorted(collected), delivered=sorted(delivered),
                  path_length_m=path_length, visited_pct=observation["exploration"]["visited_pct"] if observation else 0, stop_flushed=flushed)
        send({"type": "end", "outcome": outcome, "observed": sorted(known), "delivered": sorted(delivered)})
        robot.step(16)
        print(f"Mars mission: {outcome}; logs: {directory}", flush=True)
    except Exception as exc:
        commands = drive.command("stop", 0)
        if execution:
            execution.command(commands)
        finish_action("controller_error")
        robot.step(0)
        log.write("end", outcome="error", error_type=type(exc).__name__, simulation_s=robot.getTime())
        send({"type": "end", "outcome": "error", "observed": sorted(known), "delivered": sorted(delivered)})
        robot.step(16)
        raise
    finally:
        log.file.close()


if __name__ == "__main__":
    run()
