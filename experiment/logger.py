"""One JSONL file and explicit source snapshots per run; never overwrites a run."""

import hashlib
import json
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4
from typing import Any

from experiment.config import Config


class RunLogger:
    def __init__(self, root: Path, config: Config):
        self.run_id = uuid4().hex
        timestamp = datetime.now().astimezone().strftime("%Y-%m-%d_%H-%M-%S")
        results = root / config.results_dir
        results.mkdir(parents=True, exist_ok=True)
        suffix = 0
        while True:
            name = timestamp if suffix == 0 else f"{timestamp}_{suffix}"
            self.directory = results / name
            try:
                self.directory.mkdir()
                break
            except FileExistsError:
                suffix += 1
        self.context = {
            "schema_version": 1,
            "experiment_id": config.experiment_id,
            "run_id": self.run_id,
            "scenario_id": config.scenario_id,
            "seed": config.seed,
        }
        self.file = (self.directory / "steps.jsonl").open("x", encoding="utf-8")

    def snapshot(self, root: Path, config: Config) -> dict[str, str]:
        paths = [root / "worlds/apartment.wbt"]
        paths += sorted((root / "controllers/cleaner").rglob("*.py"))
        paths += sorted((root / "experiment").rglob("*.py"))
        hashes = {}
        for path in paths:
            relative = path.relative_to(root)
            data = path.read_bytes()
            target = self.directory / "snapshot" / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
            hashes[str(relative)] = hashlib.sha256(data).hexdigest()
        (self.directory / "config.json").write_text(
            json.dumps(asdict(config), indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
        return hashes

    def write(self, kind: str, **data: Any) -> None:
        record = {
            **self.context,
            "record_type": kind,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            **data,
        }
        self.file.write(json.dumps(record, allow_nan=False) + "\n")
        self.file.flush()

    def close(self) -> None:
        self.file.close()
