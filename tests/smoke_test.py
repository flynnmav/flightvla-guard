"""Smoke test: run the full pipeline and assert the safety/physics invariants.

  python tests/smoke_test.py
"""
from __future__ import annotations

import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from flightvla.faults import FaultSchedule          # noqa: E402
from flightvla.metrics import compute_metrics       # noqa: E402
from flightvla.schema import ActionBlock, SchemaError  # noqa: E402
from flightvla.sim import Runner                    # noqa: E402
from flightvla.world import create_task             # noqa: E402


def run(agent, vehicle, faults, seed=7):
    scene = create_task("valve-inspection")
    rec = Runner(agent, vehicle, scene, FaultSchedule(faults, duration=scene.duration),
                 seed=seed).run()
    rec["metrics"] = compute_metrics(rec, scene)
    return rec, scene


def check(name, cond, detail=""):
    status = "PASS" if cond else "FAIL"
    print(f"  [{status}] {name}" + (f"  ({detail})" if detail else ""))
    return cond


def main() -> int:
    ok = True
    print("== schema ==")
    try:
        ActionBlock.from_json('{"frame":"body","horizon":2,"dt":0.2,'
                              '"delta_position":[[9,0,0],[0,0,0]],'
                              '"delta_orientation":[[0,0,0],[0,0,0]],'
                              '"stop_probability":[0,0],"confidence":0.5}')
        ok = check("oversized delta rejected by schema", False)
    except SchemaError:
        ok = check("oversized delta rejected by schema", True)
    try:
        ActionBlock.from_json('{"frame":"body","horizon":1,"dt":0.2,'
                              '"delta_position":[[0.1,0,0]],'
                              '"delta_orientation":[[0,0,0]],'
                              '"stop_probability":[0],"confidence":0.5,'
                              '"actuator_command":[0.3,0.3,0.3,0.3]}')
        ok = check("actuator fields ignored (unrepresentable in schema)", True)
    except SchemaError:
        ok = check("actuator fields ignored", False)

    print("== clean run (no faults) ==")
    rec_q, scene = run("smolvla", "quad", "")
    rec_o, _ = run("smolvla", "omni-hex", "")
    mq, mo = rec_q["metrics"], rec_o["metrics"]
    ok &= check("quad completes task", mq["task_success"], str(mq["success_t"]))
    ok &= check("omni completes task", mo["task_success"], str(mo["success_t"]))
    ok &= check("no executed zone incursions (quad)",
                mq["executed_zone_incursions"] == 0, str(mq["executed_zone_incursions"]))
    ok &= check("no executed zone incursions (omni)",
                mo["executed_zone_incursions"] == 0, str(mo["executed_zone_incursions"]))
    ok &= check("guard intervenes on raw plans (quad)", mq["interventions_total"] > 3,
                str(mq["interventions_total"]))
    ok &= check("min executed distance >= 1.5m - eps (quad)",
                mq["min_dist_gauge_executed"] >= 1.45, str(mq["min_dist_gauge_executed"]))
    ok &= check("min executed distance >= 1.5m - eps (omni)",
                mo["min_dist_gauge_executed"] >= 1.45, str(mo["min_dist_gauge_executed"]))
    ok &= check("clean run has zero timeouts",
                mq["latency_timeouts"] == 0 and mo["latency_timeouts"] == 0)

    print("== quad vs omni physics story ==")
    ok &= check("omni camera stays framed better than quad during transit",
                mo["bore_rms_transit_deg"] < mq["bore_rms_transit_deg"],
                f"omni {mo['bore_rms_transit_deg']} deg vs quad {mq['bore_rms_transit_deg']} deg")

    print("== faulted run (latency-300ms, gust, visual-loss) ==")
    rec_f, _ = run("smolvla", "omni-hex", "latency-300ms,gust,visual-loss")
    mf = rec_f["metrics"]
    ok &= check("faulted run still completes (faults stage after nominal)",
                mf["task_success"], str(mf["success_t"]))
    ok &= check("latency fault produces timeouts", mf["latency_timeouts"] > 0,
                str(mf["latency_timeouts"]))
    ok &= check("faulted run still safe", mf["executed_zone_incursions"] == 0
                and mf["min_dist_gauge_executed"] >= 1.45)

    print("== report build ==")
    from flightvla.report import build_report
    rep = build_report([rec_q, rec_o], scene)
    ok &= check("compare table has rows", len(rep["compare"]) > 5)
    ok &= check("runs embedded", len(rep["runs"]) == 2)

    print()
    print("SMOKE " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
