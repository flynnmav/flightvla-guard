"""In-process simulation backend (the built-in evaluation vehicle)."""
from __future__ import annotations

from typing import List, Optional

from .base import OffboardBackend


class SimBackend(OffboardBackend):
    def __init__(self, stream_rate_hz: float = 50.0):
        self.stream_rate_hz = stream_rate_hz
        self._started = False
        self._last_stream_t = -1e9
        self.n_setpoints = 0

    def start(self) -> None:
        self._started = True

    def stop(self) -> None:
        self._started = False

    def stream_setpoints(self, positions: List[List[float]], dts: List[float],
                          yaw: Optional[float] = None) -> None:
        self.n_setpoints += len(positions)
        self._last_stream_t = 0.0  # sim clock is managed by the runner

    def stream_hold(self, position: List[float], dt: float) -> None:
        self.n_setpoints += int(dt * self.stream_rate_hz)
        self._last_stream_t = 0.0

    @property
    def stream_healthy(self) -> bool:
        return self._started
