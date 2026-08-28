"""OffboardBackend contract (see module docstring of the package for the PX4 story)."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Optional


class OffboardBackend(ABC):
    """A one-way setpoint stream plus a state feedback channel."""

    @abstractmethod
    def start(self) -> None:
        """Arm offboard mode and begin the heartbeat."""

    @abstractmethod
    def stop(self) -> None:
        """Leave offboard cleanly (PX4 failsafe takes over if we vanish)."""

    @abstractmethod
    def stream_setpoints(self, positions: List[List[float]], dts: List[float],
                          yaw: Optional[float] = None) -> None:
        """Push the next chunk of guard-approved setpoints at the offboard rate."""

    @abstractmethod
    def stream_hold(self, position: List[float], dt: float) -> None:
        """Stream a hold-in-place setpoint (keeps the offboard heartbeat alive)."""

    @property
    @abstractmethod
    def stream_healthy(self) -> bool:
        """False when the guard failed to feed the stream — triggers fallback."""
