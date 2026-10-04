"""The requested English Laya checkpoint; no automatic switch to other checkpoints."""

import os

from .system_one import SystemOneBrain


class LayaBrain(SystemOneBrain):
    def __init__(self, timeout_s: float = 2.0):
        super().__init__("laya", os.environ.get("LAYA_BASE_URL", "http://127.0.0.1:8009"), "laya",
                         os.environ.get("LOCAL_MODEL_API_KEY", ""), timeout_s)
