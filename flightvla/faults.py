"""Fault injection: latency, gusts, visual loss, offboard stream loss.

Faults are declared in the CLI, e.g.  --fault latency-300ms,gust,visual-loss
and evaluated per simulation tick. `windows` is exported to the report so the
HTML timeline can shade exactly when each fault was active.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class FaultState:
    latency_bias: float = 0.0          # seconds added to agent inference
    gust_accel: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    visual_loss: bool = False
    offboard_loss: bool = False


class FaultSchedule:
    """Parses a comma-separated fault spec and evaluates it over time.

    Faults are STAGED relative to the run duration so a single episode shows
    nominal flight first and the fault afterwards (the demo narrative):
      gust          ~ 55%-63% of the run
      visual-loss   ~ 72%-80%
      latency       active from 75% onwards
      offboard-loss from 82% onwards
    """

    KNOWN = ("latency-300ms", "latency-500ms", "gust", "visual-loss", "offboard-loss")

    def __init__(self, spec: str = "", duration: float = 30.0):
        self.spec = spec.strip()
        self.duration = duration
        self.names: List[str] = []
        if self.spec:
            for tok in self.spec.split(","):
                tok = tok.strip().lower()
                if not tok:
                    continue
                if tok not in self.KNOWN:
                    raise ValueError(
                        f"unknown fault {tok!r}; available: {', '.join(self.KNOWN)}")
                self.names.append(tok)
        D = duration
        self._gust: Optional[Tuple[float, float]] = (
            (0.55 * D, 0.63 * D) if "gust" in self.names else None)
        self._visual: Optional[Tuple[float, float]] = (
            (0.72 * D, 0.80 * D) if "visual-loss" in self.names else None)
        self._latency_start: float = 0.75 * D if (
            "latency-300ms" in self.names or "latency-500ms" in self.names) else None
        self._offboard: Optional[Tuple[float, float]] = (
            (0.82 * D, D) if "offboard-loss" in self.names else None)

    # ------------------------------------------------------------------ #
    def evaluate(self, t: float) -> FaultState:
        st = FaultState()
        if self._latency_start is not None and t >= self._latency_start:
            st.latency_bias = 0.500 if "latency-500ms" in self.names else 0.300
        if self._gust:
            t0, t1 = self._gust
            if t0 <= t <= t1:
                s = math.sin(math.pi * (t - t0) / (t1 - t0))  # smooth in/out
                st.gust_accel = (3.5 * s, 0.0, 0.0)
        if self._visual:
            st.visual_loss = self._visual[0] <= t <= self._visual[1]
        if self._offboard:
            st.offboard_loss = t >= self._offboard[0]
        return st

    def windows(self) -> List[dict]:
        out = []
        if self._gust:
            out.append({"kind": "gust", "t0": self._gust[0], "t1": self._gust[1]})
        if self._visual:
            out.append({"kind": "visual-loss", "t0": self._visual[0], "t1": self._visual[1]})
        if self._latency_start is not None:
            out.append({"kind": "latency", "t0": self._latency_start, "t1": self.duration})
        if self._offboard:
            out.append({"kind": "offboard-loss", "t0": self._offboard[0], "t1": self._offboard[1]})
        return out

    def to_dict(self) -> dict:
        return {"spec": self.spec, "names": self.names, "windows": self.windows()}
