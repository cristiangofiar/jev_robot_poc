"""Simulator-independent brain adapters."""

from .base import Brain
from experiment.config import Config


def create_brain(config: Config) -> Brain:
    if config.brain == "rules":
        from .rules import RuleBasedBrain
        return RuleBasedBrain(config.stop_distance_m, config.seed, config.decision_interval_ms)
    if config.brain == "threshold":
        from .rules import ThresholdBrain
        return ThresholdBrain(config.stop_distance_m)
    if config.brain == "jev":
        from .jev import JevBrain
        return JevBrain(config.decision_timeout_s)
    if config.brain == "kev":
        from .kev import KevBrain
        return KevBrain(config.decision_timeout_s)
    if config.brain == "laya":
        from .laya import LayaBrain
        return LayaBrain(config.decision_timeout_s)
    raise ValueError(f"Unknown brain: {config.brain}")
