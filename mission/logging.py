"""Explicit source snapshots and JSONL; no environment or endpoint dumps."""
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from time import perf_counter


class Log:
    def __init__(self, directory, filename="steps.jsonl"):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.file = (self.directory / filename).open("x", encoding="utf-8")
        self.started = perf_counter()

    def write(self, kind, **data):
        self.file.write(json.dumps({"type": kind, "utc": datetime.now(timezone.utc).isoformat(),
                                   "wall_s": perf_counter() - self.started, **data}, allow_nan=False) + "\n")
        self.file.flush()

    def snapshot(self, root):
        paths = list((root / "mission").glob("*.py")) + list((root / "controllers").rglob("*.py"))
        paths += list(root.glob("*.py")) + [root / "worlds/mars.wbt"]
        manifest = root / ".local_models/manifest.json"
        if manifest.is_file():
            paths.append(manifest)
        hashes = {}
        for path in paths:
            relative = path.relative_to(root)
            data = path.read_bytes()
            target = self.directory / "snapshot" / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
            hashes[relative.as_posix()] = hashlib.sha256(data).hexdigest()
        return hashes
