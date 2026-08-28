"""Vehicle registry: airframes the guard knows how to reason about."""
from __future__ import annotations

from .base import G, Limits, Vehicle
from .omni import OmniVehicle
from .quad import QuadVehicle

VEHICLES = {
    "quad": QuadVehicle,
    "quad-x": QuadVehicle,
    "omni-hex": lambda: OmniVehicle("omni-hex", rotors=6),
    "omni-octo": lambda: OmniVehicle("omni-octo", rotors=8),
}


def create_vehicle(name: str) -> Vehicle:
    key = name.strip().lower()
    if key not in VEHICLES:
        raise ValueError(
            f"unknown vehicle {name!r}; available: {', '.join(sorted(VEHICLES))}")
    return VEHICLES[key]()
