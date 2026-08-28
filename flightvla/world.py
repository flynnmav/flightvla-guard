"""Task scenes: geometry, mission instruction, and the constraint set the guard enforces.

The flagship task `valve-inspection` encodes the demo instruction literally:

    找到红色压力表，从左侧靠近并检查；保持相机始终正对仪表；距离不得小于 1.5 米；
    不要进入黄色区域；看不清时立即悬停。

Each clause becomes a machine-checkable constraint so the report can show,
per constraint, how the guard enforced it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

DEMO_INSTRUCTION = (
    "找到红色压力表，从左侧靠近并检查；保持相机始终正对仪表；"
    "距离不得小于 1.5 米；不要进入黄色区域；看不清时立即悬停。"
)


@dataclass
class KeepOutCylinder:
    cx: float
    cy: float
    r: float
    z0: float = 0.0
    z1: float = 3.0
    label: str = "yellow zone"

    def horizontal_distance(self, p: Tuple[float, float, float]) -> float:
        return math.hypot(p[0] - self.cx, p[1] - self.cy)

    def contains(self, p: Tuple[float, float, float], margin: float = 0.0) -> bool:
        return (self.z0 - margin <= p[2] <= self.z1 + margin
                and self.horizontal_distance(p) < self.r + margin)

    def project_out(self, p: List[float], margin: float) -> bool:
        """Push p radially outside the cylinder (in the horizontal plane).
        Returns True if p was modified."""
        d = math.hypot(p[0] - self.cx, p[1] - self.cy)
        r_min = self.r + margin
        if d >= r_min:
            return False
        if d < 1e-6:
            # exactly on the axis: push towards +x
            p[0] = self.cx + r_min
            return True
        s = r_min / d
        p[0] = self.cx + (p[0] - self.cx) * s
        p[1] = self.cy + (p[1] - self.cy) * s
        return True

    def to_dict(self) -> dict:
        return {"cx": self.cx, "cy": self.cy, "r": self.r,
                "z0": self.z0, "z1": self.z1, "label": self.label}


@dataclass
class Scene:
    name: str
    instruction: str
    constraints: List[Dict[str, str]]
    wall_x: float                 # wall plane the gauge is mounted on
    gauge: List[float]            # gauge centre [x, y, z]
    gauge_radius: float
    keepout: KeepOutCylinder
    fence_min: List[float]        # geofence box
    fence_max: List[float]
    home: List[float]
    standoff: List[float]         # goal inspection position (gauge left side)
    duration: float
    chunk_horizon: int
    chunk_dt: float
    agent_budget: float           # seconds of agent+guard latency considered on-time
    standoff_tol: float = 0.35    # metres
    on_target_angle_deg: float = 18.0
    hold_success_s: float = 3.0
    min_distance: float = 1.5     # instructed minimum distance to the target, metres

    def constraints_min_distance(self) -> float:
        return self.min_distance

    # ------------------------------------------------------------------ #
    def distance_to_gauge(self, p: List[float]) -> float:
        return math.sqrt(sum((p[i] - self.gauge[i]) ** 2 for i in range(3)))

    def clamp_to_fence(self, p: List[float], margin: float = 0.0) -> bool:
        changed = False
        for i in range(3):
            lo, hi = self.fence_min[i] + margin, self.fence_max[i] - margin
            if p[i] < lo:
                p[i] = lo
                changed = True
            elif p[i] > hi:
                p[i] = hi
                changed = True
        return changed

    def project_out_of_keepout(self, p: List[float], margin: float) -> bool:
        return self.keepout.project_out(p, margin)

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "instruction": self.instruction,
            "constraints": self.constraints,
            "wall_x": self.wall_x,
            "gauge": self.gauge,
            "gauge_radius": self.gauge_radius,
            "keepout": self.keepout.to_dict(),
            "fence_min": self.fence_min,
            "fence_max": self.fence_max,
            "home": self.home,
            "standoff": self.standoff,
            "duration": self.duration,
            "chunk_horizon": self.chunk_horizon,
            "chunk_dt": self.chunk_dt,
            "agent_budget": self.agent_budget,
        }


def valve_inspection() -> Scene:
    """The flagship demo task: inspect a red pressure valve from its left side."""
    return Scene(
        name="valve-inspection",
        instruction=DEMO_INSTRUCTION,
        constraints=[
            {"id": "approach-left", "text": "从左侧靠近仪表", "enforced_by": "agent plan + guard keep-out"},
            {"id": "face-target", "text": "保持相机正对仪表", "enforced_by": "orientation command + vehicle feasibility"},
            {"id": "min-distance", "text": "距离不得小于 1.5 m", "enforced_by": "guard standoff sphere (1.5 m + 0.25 m margin)"},
            {"id": "keep-out", "text": "不进入黄色区域", "enforced_by": "guard zone projection (+0.30 m margin)"},
            {"id": "hover-on-uncertain", "text": "看不清时立即悬停", "enforced_by": "stop_probability + guard image/timeout checks"},
        ],
        wall_x=6.5,
        gauge=[6.5, 0.0, 1.5],
        gauge_radius=0.12,
        keepout=KeepOutCylinder(cx=3.0, cy=1.1, r=1.4, z0=0.0, z1=3.0),
        fence_min=[-2.0, -6.0, 0.3],
        fence_max=[10.0, 6.0, 4.0],
        home=[0.0, 0.0, 1.5],
        standoff=[5.0, 0.9, 1.5],
        duration=30.0,
        chunk_horizon=8,
        chunk_dt=0.2,
        agent_budget=0.40,
    )


TASKS = {
    "valve-inspection": valve_inspection,
}


def create_task(name: str) -> Scene:
    key = name.strip().lower()
    if key not in TASKS:
        raise ValueError(f"unknown task {name!r}; available: {', '.join(sorted(TASKS))}")
    return TASKS[key]()
