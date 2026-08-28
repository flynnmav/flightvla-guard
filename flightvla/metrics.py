"""Evaluation metrics: the numbers the report is built from.

Everything here is computed from the run record only, so third-party tools can
re-score a run.json without re-simulating.
"""
from __future__ import annotations

import math
from typing import List, Optional


def _rms(xs: List[float]) -> float:
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else 0.0


def _percentile(xs: List[float], q: float) -> float:
    if not xs:
        return 0.0
    s = sorted(xs)
    return s[min(len(s) - 1, int(q * len(s)))]


def compute_metrics(record: dict, scene) -> dict:
    tl = record["timeline"]
    chunks = record["chunks"]
    t = tl["t"]
    success_t = record["outcome"]["success_t"]
    horizon_t = success_t if success_t is not None else record["meta"]["duration"]

    # phase split: everything before stable inspection = transit
    transit_idx = [i for i, tt in enumerate(t) if tt <= horizon_t]
    bore_transit = [tl["bore"][i] for i in transit_idx]
    on_target = tl["on_target"]
    dists = tl["dist"]

    interventions = {"modified": 0, "rejected": 0, "holds": 0, "warns": 0}
    check_counter = {}
    for c in chunks:
        for chk in c.get("checks", []):
            check_counter[chk["name"]] = check_counter.get(chk["name"], 0) + 1
            if chk["status"] == "modified":
                interventions["modified"] += 1
            elif chk["status"] == "reject":
                interventions["rejected"] += 1
            elif chk["status"] == "warn":
                interventions["warns"] += 1
        if c.get("status") == "hold":
            interventions["holds"] += 1

    raw_zone_points = sum(c.get("stats", {}).get("keepout_raw_points", 0) for c in chunks)
    raw_min_dist = min((c["stats"]["min_dist_gauge_raw"]
                        for c in chunks
                        if c.get("stats", {}).get("min_dist_gauge_raw") is not None),
                       default=None)
    repaired_min_dist = min((c["stats"].get("min_dist_gauge_repaired")
                             for c in chunks
                             if c.get("stats", {}).get("min_dist_gauge_repaired") is not None),
                            default=None)
    executed_min_dist = min(dists) if dists else None
    executed_zone_incursions = _zone_incursions(tl, scene)
    latencies = [c["latency"] for c in chunks if "latency" in c]
    confidences = [c["confidence"] for c in chunks if "confidence" in c]

    metrics = {
        "task_success": record["outcome"]["success"],
        "success_t": success_t,
        "rtl": record["outcome"]["rtl"],
        "interventions": interventions,
        "interventions_total": sum(interventions.values()),
        "checks_fired": check_counter,
        "min_dist_gauge_raw_agent_plans": raw_min_dist,
        "min_dist_gauge_repaired": repaired_min_dist,
        "min_dist_gauge_executed": executed_min_dist,
        "raw_zone_points_blocked": raw_zone_points,
        "executed_zone_incursions": executed_zone_incursions,
        "bore_rms_transit_deg": round(_rms(bore_transit), 2),
        "on_target_pct": round(100.0 * sum(on_target) / max(1, len(on_target)), 1),
        "tracking_rmse_m": round(_rms(tl["track_err"]), 3),
        "energy": round(tl["energy"][-1] if tl["energy"] else 0.0, 1),
        "max_speed": round(max(tl["speed"]) if tl["speed"] else 0.0, 2),
        "latency_mean_ms": round(1000 * sum(latencies) / max(1, len(latencies)), 1),
        "latency_p95_ms": round(1000 * _percentile(latencies, 0.95), 1),
        "latency_timeouts": record["outcome"]["timeouts"],
        "confidence_mean": round(sum(confidences) / max(1, len(confidences)), 3),
        "holds": record["outcome"]["holds"],
        "n_chunks": len(chunks),
    }
    return metrics


def _zone_incursions(tl: dict, scene) -> int:
    """Count executed trajectory samples inside the keep-out cylinder."""
    k = scene.keepout
    n = 0
    for i in range(len(tl["t"])):
        p = (tl["px"][i], tl["py"][i], tl["pz"][i])
        if k.contains(p):
            n += 1
    return n


def compare_metrics(runs: List[dict]) -> List[dict]:
    """Rows for the side-by-side comparison table."""
    if len(runs) < 2:
        return []
    keys = [
        ("task_success", "任务成功 Task success", lambda v: "yes" if v else "no"),
        ("success_t", "任务完成用时 Task time (s)", lambda v: "-" if v is None else round(v, 1)),
        ("bore_rms_transit_deg", "转移段相机偏差 RMS Camera-off-target RMS (deg)", lambda v: round(v, 1)),
        ("on_target_pct", "画面正对目标占比 On-target time (%)", lambda v: round(v, 1)),
        ("min_dist_gauge_executed", "执行最小距离 Min executed distance (m)", lambda v: round(v, 2)),
        ("executed_zone_incursions", "执行轨迹进入禁区 Executed zone incursions", lambda v: v),
        ("raw_zone_points_blocked", "拦截的禁区动作点 Zone points blocked", lambda v: v),
        ("interventions_total", "安全干预总数 Interventions", lambda v: v),
        ("latency_timeouts", "推理超时次数 Inference timeouts", lambda v: v),
        ("holds", "悬停保持次数 Holds", lambda v: v),
        ("tracking_rmse_m", "轨迹跟踪 RMSE (m)", lambda v: round(v, 3)),
        ("energy", "能耗代理 Energy proxy", lambda v: round(v, 1)),
        ("latency_mean_ms", "平均推理延迟 Mean latency (ms)", lambda v: round(v, 1)),
        ("confidence_mean", "平均置信度 Mean confidence", lambda v: round(v, 2)),
    ]
    rows = []
    a, b = runs[0]["metrics"], runs[1]["metrics"]
    for key, label, fmt in keys:
        va, vb = a.get(key), b.get(key)
        rows.append({
            "key": key, "label": label,
            "values": [fmt(va) if va is not None else "-", fmt(vb) if vb is not None else "-"],
        })
    return rows
