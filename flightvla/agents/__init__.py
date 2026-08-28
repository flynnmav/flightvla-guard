"""Flight agent registry. Add real VLA adapters here (SmolVLA, OpenVLA, ...)."""
from __future__ import annotations

from .base import AGENTS, FlightAgent, Observation, Proposal, create_agent, register
from .scripted import ScriptedVLA

# keep a stable public set even if only one built-in agent ships today
BUILTINS = [ScriptedVLA]
