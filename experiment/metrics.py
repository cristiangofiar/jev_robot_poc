"""Initial run metrics. Do not turn missing ground truth or incomplete runs into success."""

import csv
import hashlib
import json
import re
from collections import defaultdict
from math import dist
from pathlib import Path
from statistics import mean


def read_records(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low = int(index)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def summarize(directory: Path) -> dict:
    records = read_records(directory / "steps.jsonl")
    if not records or records[0].get("record_type") != "run_start":
        raise ValueError("missing run_start")
    start, end = records[0], records[-1]
    config = json.loads((directory / "config.json").read_text())
    steps = [r for r in records if r["record_type"] == "step"]
    reasons = []
    envelope = ("run_id", "scenario_id", "seed", "model", "model_version", "experiment_id")
    if any(any(r.get(key) != start.get(key) for key in envelope) for r in records):
        reasons.append("inconsistent_run_context")
    if config != start.get("parameters"):
        reasons.append("config_mismatch")
    if (end.get("record_type") != "run_end" or end.get("status") != "duration_reached"
            or end.get("stop_command_flushed") is not True):
        reasons.append("incomplete_controller")
    if (not steps or end.get("steps") != len(steps) or
            any(r.get("step_index") != i for i, r in enumerate(steps))):
        reasons.append("missing_steps")
    if end.get("simulation_time_s", -1) + 1e-9 < config["max_simulation_time_s"]:
        reasons.append("short_horizon")
    ground_truth = read_records(directory / "ground_truth.jsonl") if (directory / "ground_truth.jsonl").is_file() else []
    gt_end = ground_truth[-1] if ground_truth else {}
    event = next((r for r in ground_truth if r["record_type"] == "event_released"), None)
    falling = start["scenario_id"] == "falling_object_close"
    if falling:
        if (not ground_truth or ground_truth[0].get("record_type") != "scenario_start"
                or gt_end.get("record_type") != "scenario_end" or gt_end.get("status") != "duration_reached"
                or gt_end.get("event_released") is not True or event is None
                or type(gt_end.get("collision_with_spawned_object")) is not bool
                or gt_end.get("simulation_time_s", -1) + 1e-9 < config["max_simulation_time_s"]):
            reasons.append("incomplete_ground_truth")
    if any(any(r.get(key) != start.get(key) for key in envelope) for r in ground_truth):
        reasons.append("ground_truth_context_mismatch")
    controlled = {k: v for k, v in config.items() if k not in ("brain", "seed", "experiment_id", "results_dir", "batch_mode")}
    protocol_hash = hashlib.sha256(json.dumps(controlled, sort_keys=True).encode()).hexdigest()
    artifact_hash = hashlib.sha256(json.dumps(start.get("model_artifact"), sort_keys=True).encode()).hexdigest()
    # Seeds differ by design; preserve the remaining geometry and all sources.
    world = (directory / "snapshot/worlds/apartment.wbt").read_text()
    normalized_world = re.sub(r"(?m)^(\s*randomSeed\s+)\d+\s*$", r"\g<1>SEED", world)
    sources = {k: v for k, v in start.get("source_sha256", {}).items()
               if k != "worlds/apartment.wbt" and not k.startswith(".local_models/")}
    sources["normalized_world"] = hashlib.sha256(normalized_world.encode()).hexdigest()
    pipeline_hash = hashlib.sha256(json.dumps(sources, sort_keys=True).encode()).hexdigest()
    decisions = [r["decision"] for r in steps if r.get("decision") is not None]
    successful = [d for d in decisions if not d.get("metadata", {}).get("fallback")]
    latencies = [d["latency_ms"] for d in successful]
    errors = [d.get("metadata", {}).get("error_type") for d in decisions if d.get("metadata", {}).get("fallback")]
    requested = end.get("decisions", sum(r["record_type"] == "decision_request" for r in records))
    positions = [r["sensors"].get("position_m") for r in steps]
    travelled = sum(dist(a, b) for a, b in zip(positions, positions[1:]) if a is not None and b is not None)
    event_time = event["release_command_time_s"] if event else None
    perception_time = next((r["simulation_time_s"] for r in steps if event_time is not None
                            and r["simulation_time_s"] >= event_time and r["sensor_observation"].get("path_blocked") is True), None)
    # Require a NEW command transition, not a STOP already held before release.
    reaction = next((r for previous, r in zip(steps, steps[1:]) if event_time is not None
                     and r["simulation_time_s"] >= event_time
                     and previous["applied_action"] not in ("STOP", "SLOW_DOWN", "WAIT", "REPLAN")
                     and r["applied_action"] in ("STOP", "SLOW_DOWN", "WAIT", "REPLAN")), None)
    source = None
    if reaction:
        source = "safety" if reaction["safety_override"] else (
            "fallback_or_stale" if reaction.get("waiting_for_fresh_decision") or
            (reaction.get("decision") or {}).get("metadata", {}).get("fallback") else "model")
    costs = [d.get("metadata", {}).get("estimated_cost_usd") for d in successful]
    revisions = sorted({d.get("metadata", {}).get("served_model") for d in successful
                        if d.get("metadata", {}).get("served_model")})
    return {
        "directory": str(directory), "run_id": start["run_id"], "model": start["model"],
        "model_version": ";".join(revisions) or start["model_version"], "scenario": start["scenario_id"],
        "seed": start["seed"], "protocol_sha256": protocol_hash,
        "model_artifact_sha256": artifact_hash,
        "pipeline_sha256": pipeline_hash,
        "source_sha256": hashlib.sha256(json.dumps(start.get("source_sha256", {}), sort_keys=True).encode()).hexdigest(),
        "valid": not reasons, "invalid_reason": ";".join(reasons),
        "collision_with_spawned_object": gt_end.get("collision_with_spawned_object") if falling and not reasons else None,
        "bumper_contact": end.get("bumper_contact_detected"),
        "minimum_center_distance_m": gt_end.get("minimum_center_distance_m") if not reasons else None,
        "decision_requests": requested, "decision_results": len(decisions),
        "successful_responses": len(successful), "fallbacks": len(errors),
        "timeouts": sum(e in ("TimeoutError", "deadline_exceeded") for e in errors),
        "stale_responses": errors.count("stale_observation"),
        "pending_at_end": max(0, requested - len(decisions)),
        "inference_latency_mean_ms": mean(latencies) if latencies else None,
        "inference_latency_p50_ms": percentile(latencies, .5), "inference_latency_p95_ms": percentile(latencies, .95),
        "safety_override_steps": sum(bool(r["safety_override"]) for r in steps),
        "action_switches": sum(a["applied_action"] != b["applied_action"] for a, b in zip(steps, steps[1:])),
        "commanded_stationary_fraction": (sum(r["applied_action"] in ("STOP", "WAIT", "REPLAN") for r in steps) / len(steps)) if steps else None,
        "distance_travelled_m": travelled if positions and all(p is not None for p in positions) else None,
        "event_time_s": event_time, "first_blocked_sensor_time_s": perception_time,
        "event_to_command_transition_s": reaction["simulation_time_s"] - event_time if reaction else None,
        "command_transition_source": source,
        "estimated_cost_usd": sum(costs) if costs and all(c is not None for c in costs) and len(successful) == requested else None,
        "action_accuracy": None, "ece": None, "brier_score": None,
        "quality_label_status": "no_per_decision_labels",
    }


def write_csv(path: Path, rows: list[dict]):
    if not rows:
        return
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def analyze(path: Path) -> list[dict]:
    rows = []
    files = [path / "steps.jsonl"] if (path / "steps.jsonl").is_file() else sorted(path.rglob("steps.jsonl"))
    for file in files:
        try:
            rows.append(summarize(file.parent))
        except (ValueError, KeyError, TypeError, OSError) as exc:
            rows.append({"directory": str(file.parent), "valid": False, "invalid_reason": f"malformed_run:{type(exc).__name__}"})
    plan_path = path / "plan.json"
    if plan_path.is_file():
        for run in json.loads(plan_path.read_text())["runs"]:
            if not any(row.get("seed") == run["seed"] and row.get("model") == run["brain"] for row in rows):
                rows.append({"seed": run["seed"], "model": run["brain"], "valid": False,
                             "invalid_reason": "missing_recorded_run"})
    groups = defaultdict(list)
    for row in rows:
        groups[(row.get("model"), row.get("model_version"), row.get("scenario"), row.get("protocol_sha256"),
                row.get("model_artifact_sha256"), row.get("pipeline_sha256"))].append(row)
    aggregate = []
    for (model, version, scenario, protocol, artifact, pipeline), group in groups.items():
        valid = [row for row in group if row["valid"]]
        observed = [row["collision_with_spawned_object"] for row in valid if row.get("collision_with_spawned_object") is not None]
        latencies = [row["inference_latency_mean_ms"] for row in valid if row.get("inference_latency_mean_ms") is not None]
        aggregate.append({"model": model, "model_version": version, "scenario": scenario, "protocol_sha256": protocol,
                          "model_artifact_sha256": artifact,
                          "pipeline_sha256": pipeline,
                          "runs": len(group), "valid_runs": len(valid), "invalid_runs": len(group) - len(valid),
                          "collision_observed_runs": len(observed), "collision_rate": mean(observed) if observed else None,
                          "mean_run_inference_latency_ms": mean(latencies) if latencies else None})
    path.mkdir(parents=True, exist_ok=True)
    write_csv(path / "metrics.csv", rows)
    write_csv(path / "aggregate.csv", aggregate)
    return rows
