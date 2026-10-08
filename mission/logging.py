"""Explicit source snapshots and JSONL; no environment or endpoint dumps."""
import hashlib
import json
import os
import math
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from time import perf_counter
from uuid import uuid4

OUTPUT_SEPARATOR = "=" * 72


def brain_output(brain, raw, latency_ms=None, status=None, note=None):
    """The same readable response block in the server, Webots and launch terminal."""
    header = f"[{datetime.now().astimezone().strftime('%H:%M:%S')}] {brain.upper()} OUTPUT"
    if status is not None:
        header += f" | HTTP {status}"
    if latency_ms is not None:
        header += f" | {latency_ms:.1f} ms"
    if note:
        header += f" | {note}"
    lines = ["", OUTPUT_SEPARATOR, header]
    for name, answer in raw.get("answers", {}).items():
        selected = answer.get("choice", answer.get("score", answer.get("noul")))
        lines.append(f"  >>> {name}: {selected} | confidence={answer.get('confidence')}")
    lines.extend([json.dumps(raw, indent=2, ensure_ascii=False, allow_nan=False), OUTPUT_SEPARATOR, ""])
    return "\n".join(lines)


def execution_history(entries):
    """Compact, independent context; complete results remain in the JSONL log."""
    keys = ("started_s", "requested", "accepted", "applied", "motor", "result", "safety",
            "executed_s", "displacement_m", "yaw_change_deg")
    return deepcopy([{**{key: entry[key] for key in keys},
                      **{key: entry[key] for key in ("fallback", "operation_ok", "reason")
                         if entry.get(key) is not None and (key != "reason" or entry["result"] == "failed")}}
                     for entry in entries[-2:]])


def start_run_log(directory, brain):
    """Reuse a launch directory once; UI resets get fresh sibling directories."""
    directory = Path(directory)
    while True:
        if not any((directory / name).exists() for name in ("frames", "ground_truth.jsonl")):
            try:
                return Log(directory, brain=brain)
            except FileExistsError:
                pass  # steps.jsonl was already created, possibly by another start.
        directory = directory.parent / f"{datetime.now():%Y%m%d_%H%M%S}_{brain}_{uuid4().hex[:6]}"


class ActionExecution:
    """Track motor commands separately from GPS/IMU motion and the chosen action."""
    def __init__(self, requested, accepted, fallback, now, position, yaw):
        self.position, self.yaw, self.time = position[:2], yaw, now
        self.executed_s = 0
        self.entry = {"requested": requested, "accepted": accepted, "applied": "STOP",
                      "fallback": fallback, "started_s": now, "result": "running", "safety": None,
                      "motor": {"mode": "stop", "wheel_rad_s": {"Left": 0, "Right": 0}},
                      "executed_s": 0, "displacement_m": 0, "yaw_change_deg": 0}

    def observe(self, now, position, yaw):
        if self.entry["motor"]["mode"] != "stop":
            self.executed_s += now - self.time
        self.time = now
        self.entry.update(executed_s=round(self.executed_s, 3),
                          displacement_m=round(math.dist(self.position, position[:2]), 4),
                          yaw_change_deg=round(math.degrees((yaw-self.yaw+math.pi) % (2*math.pi)-math.pi), 2))

    def command(self, commands):
        self.entry["motor"] = deepcopy({key: commands[key] for key in ("mode", "wheel_rad_s")})
        if commands["mode"] != "stop":
            self.entry["applied"] = self.entry["accepted"]

    def finish(self, reason, safety=None, operation_ok=None):
        self.entry.update(ended_s=self.time, reason=reason, safety=safety)
        if safety:
            self.entry["result"] = "interrupted" if self.executed_s > 0 else "blocked"
            if self.executed_s == 0:
                self.entry["applied"] = "STOP"
        elif operation_ok is not None:
            self.entry.update(operation_ok=operation_ok, result="completed" if operation_ok else "failed")
            if operation_ok:
                self.entry["applied"] = self.entry["accepted"]
        else:
            self.entry["result"] = "completed" if reason == "interval_elapsed" else "interrupted"
            if self.entry["accepted"].startswith(("INSPECT_", "COLLECT_")):
                self.entry["result"] = "failed"


class Log:
    def __init__(self, directory, filename="steps.jsonl", brain=None):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.file = (self.directory / filename).open("x", encoding="utf-8")
        self.started = perf_counter()
        self.brain = brain

    def write(self, kind, **data):
        record = {"type": kind, "utc": datetime.now(timezone.utc).isoformat(),
                  "wall_s": perf_counter() - self.started, **data}
        self.file.write(json.dumps(record, allow_nan=False) + "\n")
        self.file.flush()
        if self.brain and kind in ("response", "late_response"):
            decision = data["decision"]
            raw = decision.get("raw_response")
            if raw is None and decision.get("choice") is not None:
                raw = {"model": decision.get("served_model", self.brain), "answers": {"mission": {
                    "type": "choice", **{k: decision[k] for k in ("choice", "confidence", "probabilities") if k in decision}}}}
            if raw is not None:
                rejection = data.get("rejection")
                note = "late response; ignored" if kind == "late_response" else f"rejected: {rejection}" if rejection else None
                print(brain_output(self.brain, raw, data.get("latency_ms"), decision.get("http_status"), note), end="", flush=True)

    def snapshot(self, root):
        paths = list((root / "mission").glob("*.py")) + list((root / "controllers").rglob("*.py"))
        paths += list((root / "protos").rglob("*.proto"))
        paths += list((root / "tests").glob("*.py"))
        paths += list(root.glob("*.py")) + [root / "worlds/mars.wbt"]
        world = Path(os.environ.get("MARS_WORLD", str(root / "worlds/mars.wbt")))
        if world != root / "worlds/mars.wbt":
            paths.append(world)
        manifest = root / ".local_models/manifest.json"
        if manifest.is_file():
            paths.append(manifest)
        hashes = {}
        for path in paths:
            relative = path.relative_to(root) if path.is_relative_to(root) else Path("worlds/active.wbt")
            data = path.read_bytes()
            target = self.directory / "snapshot" / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
            hashes[relative.as_posix()] = hashlib.sha256(data).hexdigest()
        return hashes
