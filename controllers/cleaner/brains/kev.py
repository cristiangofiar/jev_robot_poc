"""Kev's decision head served locally by llama.cpp, using the pinned Q4_K_M export."""

import os

from .system_one import SystemOneBrain


class KevBrain(SystemOneBrain):
    def __init__(self, timeout_s: float = 2.0):
        super().__init__("kev", os.environ.get("KEV_BASE_URL", "http://127.0.0.1:8008"), "kev-4b",
                         os.environ.get("LOCAL_MODEL_API_KEY", ""), timeout_s)
