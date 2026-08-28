"""flightvla command line interface.

  flightvla run    --agent smolvla --vehicle omni-hex --task valve-inspection --fault latency-300ms
  flightvla demo   --task valve-inspection          # quad vs omni-hex side-by-side report
  flightvla validate chunk.json --vehicle quad      # run an ActionBlock through the guard
"""
from __future__ import annotations

import argparse
import json
import sys
from typing import List

from . import __version__
from .agents import create_agent
from .faults import FaultSchedule
from .metrics import compute_metrics
from .report import build_report, render
from .safety import SafetyGuard
from .schema import ActionBlock
from .sim import Runner
from .vehicles import create_vehicle
from .world import create_task


def _summary_line(metrics: dict) -> str:
    return ("success={} timeouts={} interventions={} min_dist={}m zone_incursions={} "
            "bore_rms={}deg".format(
                "yes" if metrics["task_success"] else "NO",
                metrics["latency_timeouts"], metrics["interventions_total"],
                metrics["min_dist_gauge_executed"], metrics["executed_zone_incursions"],
                metrics["bore_rms_transit_deg"]))


def _execute(agent_name: str, vehicle_name: str, task_name: str, fault_spec: str,
             seed: int, duration=None) -> tuple:
    scene = create_task(task_name)
    faults = FaultSchedule(fault_spec, duration=duration or scene.duration)
    runner = Runner(agent_name, vehicle_name, scene, faults, seed=seed, duration=duration)
    record = runner.run()
    record["metrics"] = compute_metrics(record, scene)
    return record, scene


# ------------------------------------------------------------------ #
def cmd_run(args) -> int:
    record, scene = _execute(args.agent, args.vehicle, args.task, args.fault,
                             args.seed, args.duration)
    m = record["metrics"]
    print()
    print(f"  run: agent={args.agent} vehicle={args.vehicle} task={args.task} "
          f"fault='{args.fault or '-'}' seed={args.seed}")
    print(f"  {_summary_line(m)}")
    top = [(c["t_arrival"], chk) for c in record["chunks"] for chk in c.get("checks", [])
           if chk["status"] in ("modified", "reject")]
    for t, chk in top[:5]:
        print(f"    t={t:5.2f}s  {chk['name']:12s} {chk['status']:9s} {chk['detail'][:90]}")
    if len(top) > 5:
        print(f"    ... {len(top) - 5} more interventions in the report")

    if not args.no_report:
        report = build_report([record], scene,
                              title=f"FlightVLA Guard — {args.vehicle} @ {args.task}")
        out = args.out or f"reports/run-{args.vehicle}-{args.task or 'valve-inspection'}.html"
        path = render(report, out)
        print(f"  report: {path}")
    return 0


def cmd_demo(args) -> int:
    scene = create_task(args.task)
    faults = args.faults or "latency-300ms,gust,visual-loss"
    runs = []
    for vehicle in ("quad", "omni-hex"):
        rec, sc = _execute(args.agent, vehicle, args.task, faults, args.seed, args.duration)
        print(f"  [{vehicle:9s}] {_summary_line(rec['metrics'])}")
        runs.append(rec)
    report = build_report(runs, scene, title="FlightVLA Guard — quad vs omni-hex")
    out = args.out or f"reports/demo-{args.task}.html"
    path = render(report, out)
    print(f"  demo report: {path}")
    return 0


def cmd_validate(args) -> int:
    try:
        with open(args.file, "r", encoding="utf-8") as f:
            block = ActionBlock.from_dict(json.load(f))
    except (OSError, ValueError) as e:
        print(f"  schema: REJECT — {e}")
        return 1
    print(f"  schema: PASS ({block.horizon} steps @ {block.dt}s, frame={block.frame})")
    scene = create_task(args.task)
    vehicle = create_vehicle(args.vehicle)
    vehicle.reset(list(scene.home), yaw=0.0)
    guard = SafetyGuard(scene, vehicle)
    res = guard.process(block, t_arrival=block.dt, frame_age=0.05)
    print(f"  verdict: {res.status.upper()}" + (f" -> {res.fallback}" if res.fallback else ""))
    for chk in res.checks:
        mark = {"pass": "+", "modified": "~", "reject": "x", "warn": "!"}.get(chk.status, "?")
        print(f"    [{mark}] {chk.name:15s} {chk.status:9s} {chk.detail}")
    stats = res.stats
    print(f"  stats: {json.dumps(stats)}")
    return 0 if res.ok else 2


# ------------------------------------------------------------------ #
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="flightvla",
        description="FlightVLA Guard — safety gateway + evaluation platform "
                    "between VLA/VLM/LLM flight agents and PX4.")
    p.add_argument("--version", action="version", version=f"flightvla {__version__}")
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="run one closed-loop episode and emit a report")
    r.add_argument("--agent", default="smolvla", help="flight agent (default: smolvla)")
    r.add_argument("--vehicle", default="omni-hex", help="quad | omni-hex | omni-octo")
    r.add_argument("--task", default="valve-inspection", help="task scene")
    r.add_argument("--fault", default="", dest="fault",
                   help="comma list: latency-300ms, gust, visual-loss, offboard-loss")
    r.add_argument("--seed", type=int, default=7)
    r.add_argument("--duration", type=float, default=None)
    r.add_argument("--out", default=None, help="report html path")
    r.add_argument("--no-report", action="store_true")
    r.set_defaults(func=cmd_run)

    d = sub.add_parser("demo", help="quad vs omni-hex side-by-side demo report")
    d.add_argument("--agent", default="smolvla")
    d.add_argument("--task", default="valve-inspection")
    d.add_argument("--faults", default=None,
                   help="default: latency-300ms,gust,visual-loss")
    d.add_argument("--seed", type=int, default=7)
    d.add_argument("--duration", type=float, default=None)
    d.add_argument("--out", default=None)
    d.set_defaults(func=cmd_demo)

    v = sub.add_parser("validate", help="lint one ActionBlock JSON through the guard")
    v.add_argument("file", help="path to ActionBlock JSON")
    v.add_argument("--vehicle", default="quad")
    v.add_argument("--task", default="valve-inspection")
    v.set_defaults(func=cmd_validate)
    return p


def main(argv: List[str] = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
