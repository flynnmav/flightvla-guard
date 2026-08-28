"""FlightAgent: the interface any VLA / VLM / LLM flight policy implements.

The contract is deliberately narrow:

  observe (state + perception summary)  ->  one ActionBlock  (+ simulated inference latency)

Agents never see motors, never emit motors. A real adapter (SmolVLA, OpenVLA,
a GPT-style planner ...) wraps its inference call in `propose` and returns the
block; the simulated latency lets the evaluation platform score timing behaviour
deterministically.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from ..schema import ActionBlock


@dataclass
class Observation:
    """Everything the agent is allowed to know about the world this tick."""
    t: float
    position: List[float]
    velocity: List[float]
    yaw: float                     # current vehicle yaw (rad) — needed for body-frame output
    pitch: float                   # current camera pitch (rad, positive = nose down)
    bore_error_deg: float          # camera boresight vs target
    bore_horiz_deg: float
    bore_vert_deg: float
    distance_to_target: float
    target: List[float]            # perceived position of the task target (e.g. the gauge)
    target_visible: bool
    clarity: float                 # 0..1 subjective "can I see it well" from the agent
    timeout_streak: int            # consecutive chunks that missed the latency budget
    goal: List[float]              # the inspection standoff point


@dataclass
class Proposal:
    """An agent response: the block plus when it would be ready in real life."""
    block: ActionBlock
    latency: float                 # seconds of (simulated) inference time


class FlightAgent(ABC):
    name: str = "abstract"
    description: str = ""

    @abstractmethod
    def reset(self, seed: int) -> None:
        ...

    @abstractmethod
    def propose(self, obs: Observation) -> Proposal:
        ...


AGENTS = {}


def register(cls):
    AGENTS[cls.name] = cls
    return cls


def create_agent(name: str) -> FlightAgent:
    key = name.strip().lower()
    if key not in AGENTS:
        raise ValueError(f"unknown agent {name!r}; available: {', '.join(sorted(AGENTS))}")
    return AGENTS[key]()
