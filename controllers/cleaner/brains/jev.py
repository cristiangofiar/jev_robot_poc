"""Jev through OpenRouter’s System One compatible endpoint; credentials are read from the environment."""

import os

from .system_one import SystemOneBrain, price


class JevBrain(SystemOneBrain):
    def __init__(self, timeout_s: float = 2.0):
        key = os.environ.get("OPENROUTER_API_KEY", "")
        if not key:
            raise ValueError("Fill OPENROUTER_API_KEY in the project's .env before using Jev")
        super().__init__("jev", os.environ.get("JEV_BASE_URL", "https://openrouter.ai/api"),
                         os.environ.get("JEV_MODEL", "typesafe/jev-1.13"), key, timeout_s,
                         price(os.environ.get("JEV_INPUT_USD_PER_MILLION", "")),
                         price(os.environ.get("JEV_OUTPUT_USD_PER_MILLION", "")))
