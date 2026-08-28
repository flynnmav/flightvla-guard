"""Offboard backends: how guard-approved setpoints reach a vehicle.

The runtime ships a `SimBackend`; real deployments plug in a PX4 backend
(MAVSDK / MAVROS / uXRCE-DDS). The contract mirrors the PX4 Offboard Mode
requirement: a *healthy setpoint stream* must keep flowing (PX4 demands
> 2 Hz), otherwise PX4 exits offboard and runs its configured failsafe.

That PX4 rule is exactly FlightVLA Guard's execution boundary: the guard is the
only writer of the offboard stream, so no agent output can ever reach motors
without passing checks 1-11 of the pipeline.
"""
from __future__ import annotations

from .base import OffboardBackend
from .sim import SimBackend

BACKENDS = {
    "sim": SimBackend,
}


def create_backend(name: str) -> OffboardBackend:
    key = name.strip().lower()
    if key not in BACKENDS:
        raise ValueError(f"unknown backend {name!r}; available: {', '.join(sorted(BACKENDS))}")
    return BACKENDS[key]()
