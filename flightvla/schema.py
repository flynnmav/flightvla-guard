"""ActionBlock v0.1 — the only wire format a flight agent may use to talk to FlightVLA Guard.

An ActionBlock is a short-horizon, vehicle-agnostic *motion proposal*.

It deliberately cannot express motor PWM, rotor speeds, per-rotor thrusts,
PX4 actuator commands, or raw MAVLink messages: "the agent cannot bypass the
safety layer" is enforced by the data format itself, not by convention.
Everything in here is either geometry (delta_position / delta_orientation),
self-assessment (confidence / stop_probability) or bookkeeping metadata.

See docs/action-format.md for the human-readable spec.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import List, Optional

SCHEMA_VERSION = "0.1"
ALLOWED_FRAMES = ("body", "world")

# Hard schema ceilings. The guard applies stricter, vehicle-specific limits later.
MAX_STEP_DISPLACEMENT = 2.5   # metres per step, sanity ceiling
MAX_STEP_ROTATION = 1.0       # radians per step, sanity ceiling
MAX_DT = 1.0                  # seconds per step
MAX_HORIZON = 64


class SchemaError(ValueError):
    """Raised when an agent output violates the ActionBlock schema."""


def _check_matrix(name: str, value, horizon: int, rows: int, per_elem_max: float,
                  unit: str) -> None:
    if not isinstance(value, list) or len(value) != horizon:
        raise SchemaError(
            f"{name} must be a list of length horizon={horizon}, got "
            f"{len(value) if isinstance(value, list) else type(value).__name__}")
    for k, row in enumerate(value):
        if not isinstance(row, list) or len(row) != rows:
            raise SchemaError(f"{name}[{k}] must be a list of {rows} numbers")
        for j, x in enumerate(row):
            if not isinstance(x, (int, float)) or isinstance(x, bool) or not math.isfinite(x):
                raise SchemaError(f"{name}[{k}][{j}] is not a finite number: {x!r}")
            if abs(x) > per_elem_max:
                raise SchemaError(
                    f"{name}[{k}][{j}]={x} exceeds schema ceiling "
                    f"{per_elem_max} {unit} per step")


@dataclass
class ActionBlock:
    """A short-horizon 6-DoF motion proposal emitted by a flight agent.

    delta_position[k] is the displacement for step k (metres, in `frame`).
    delta_orientation[k] is [d_yaw, d_pitch, d_roll] for step k (radians).
    stop_probability[k] is the agent's own "I am not sure / stop here" signal;
    the guard truncates the block there and holds.
    """

    frame: str                             # "body" (x fwd, y left, z up) or "world" (ENU)
    horizon: int                           # number of steps H
    dt: float                              # seconds per step
    delta_position: List[List[float]]      # [H][3]
    delta_orientation: List[List[float]]   # [H][3]
    stop_probability: List[float]          # [H]
    confidence: float                      # scalar in [0, 1]
    agent: str = "unknown"
    seq: int = 0                           # filled by the runtime
    t_created: Optional[float] = None      # sim time the block was produced

    # ------------------------------------------------------------------ #
    def validate(self) -> None:
        if self.frame not in ALLOWED_FRAMES:
            raise SchemaError(f"frame must be one of {ALLOWED_FRAMES}, got {self.frame!r}")
        if not isinstance(self.horizon, int) or not (1 <= self.horizon <= MAX_HORIZON):
            raise SchemaError(f"horizon must be an int in [1, {MAX_HORIZON}]")
        if not isinstance(self.dt, (int, float)) or not (1e-3 <= self.dt <= MAX_DT):
            raise SchemaError(f"dt must be in [0.001, {MAX_DT}] s, got {self.dt}")
        if len(self.stop_probability) != self.horizon:
            raise SchemaError("stop_probability must have length horizon")
        for k, p in enumerate(self.stop_probability):
            if not isinstance(p, (int, float)) or not (0.0 <= p <= 1.0):
                raise SchemaError(f"stop_probability[{k}] must be in [0, 1], got {p}")
        if not isinstance(self.confidence, (int, float)) or not (0.0 <= self.confidence <= 1.0):
            raise SchemaError(f"confidence must be in [0, 1], got {self.confidence}")
        _check_matrix("delta_position", self.delta_position, self.horizon, 3,
                      MAX_STEP_DISPLACEMENT, "m")
        _check_matrix("delta_orientation", self.delta_orientation, self.horizon, 3,
                      MAX_STEP_ROTATION, "rad")

    # ------------------------------------------------------------------ #
    def first_stop_step(self, threshold: float) -> Optional[int]:
        """Index of the first step whose stop_probability crosses `threshold`."""
        for k, p in enumerate(self.stop_probability):
            if p > threshold:
                return k
        return None

    def step_displacements(self) -> List[float]:
        return [math.sqrt(dx * dx + dy * dy + dz * dz)
                for dx, dy, dz in self.delta_position]

    # ------------------------------------------------------------------ #
    def to_dict(self) -> dict:
        return {
            "schema_version": SCHEMA_VERSION,
            "frame": self.frame,
            "horizon": self.horizon,
            "dt": self.dt,
            "delta_position": self.delta_position,
            "delta_orientation": self.delta_orientation,
            "stop_probability": self.stop_probability,
            "confidence": round(self.confidence, 3),
            "agent": self.agent,
            "seq": self.seq,
            "t_created": None if self.t_created is None else round(self.t_created, 3),
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)

    @staticmethod
    def from_dict(d: dict) -> "ActionBlock":
        try:
            block = ActionBlock(
                frame=d["frame"], horizon=d["horizon"], dt=d["dt"],
                delta_position=d["delta_position"],
                delta_orientation=d["delta_orientation"],
                stop_probability=d["stop_probability"],
                confidence=d["confidence"],
                agent=d.get("agent", "unknown"),
                seq=int(d.get("seq", 0)),
                t_created=d.get("t_created"),
            )
        except KeyError as e:
            raise SchemaError(f"missing required field: {e}") from None
        block.validate()
        return block

    @staticmethod
    def from_json(text: str) -> "ActionBlock":
        try:
            return ActionBlock.from_dict(json.loads(text))
        except json.JSONDecodeError as e:
            raise SchemaError(f"invalid JSON: {e}") from None


def zeros_block(frame: str, horizon: int, dt: float, agent: str,
                stop_probability: float = 0.0, confidence: float = 0.5) -> ActionBlock:
    """A hold-in-place block (used by the guard's fallback paths)."""
    return ActionBlock(
        frame=frame, horizon=horizon, dt=dt,
        delta_position=[[0.0, 0.0, 0.0] for _ in range(horizon)],
        delta_orientation=[[0.0, 0.0, 0.0] for _ in range(horizon)],
        stop_probability=[stop_probability] * horizon,
        confidence=confidence, agent=agent,
    )
