"""ScriptedVLA — a deterministic stand-in for a real VLA (default name: `smolvla`).

It behaves the way short-horizon learned policies actually behave:

  * navigates myopically toward its current waypoint with straight-line
    velocity chunks (no map of keep-out zones -> the guard must repair);
  * occasionally over-speeds or under-estimates distances;
  * biases toward the target when close (occasionally dipping inside the
    1.5 m standoff -> guard clamps);
  * self-reports low confidence / high stop_probability when its view of the
    gauge degrades -> per the user instruction, the vehicle hovers;
  * has a realistic, jittery inference latency (faults can inflate it).

Swap this class for a real SmolVLA/OpenVLA adapter to fly actual weights:
implement FlightAgent.propose() and register it — nothing else changes.
"""
from __future__ import annotations

import math
import random
from typing import List, Optional, Tuple

from ..schema import ActionBlock
from .base import FlightAgent, Observation, Proposal, register

VMAX = 2.0          # the agent's own (slightly optimistic) speed belief
CHUNK_STEPS = 8
CHUNK_DT = 0.2


@register
class ScriptedVLA(FlightAgent):
    name = "smolvla"
    description = "scripted short-horizon VLA stand-in (myopic straight-line planner)"

    def reset(self, seed: int) -> None:
        self.rng = random.Random(seed * 7919 + 13)
        self.chunk_seq = 0
        self.wp_idx = 0          # forward-only waypoint progression

    # ------------------------------------------------------------------ #
    def _clarity(self, obs: Observation) -> float:
        """View quality as reported by the perception stack (runner-computed)."""
        return obs.clarity

    def _waypoints(self, obs: Observation) -> List[List[float]]:
        # the "approach from the left" route: a high via-point, then the standoff.
        # the straight shot to the via-point cuts the yellow zone — the agent is
        # myopic about painted keep-out zones, which is exactly what the guard is for.
        return [[4.9, 2.1, 1.5], list(obs.goal)]

    # ------------------------------------------------------------------ #
    def propose(self, obs: Observation) -> Proposal:
        latency = max(0.05, self.rng.gauss(0.11, 0.03))
        clarity = self._clarity(obs)

        if not obs.target_visible or clarity < 0.32:
            # instruction: "看不清时立即悬停" — the agent says stop, loudly
            block = ActionBlock(
                frame="body", horizon=CHUNK_STEPS, dt=CHUNK_DT,
                delta_position=[[0.0, 0.0, 0.0] for _ in range(CHUNK_STEPS)],
                delta_orientation=[[0.0, 0.0, 0.0] for _ in range(CHUNK_STEPS)],
                stop_probability=[0.97] * CHUNK_STEPS,
                confidence=0.08, agent=self.name, seq=self.chunk_seq,
            )
            self.chunk_seq += 1
            return Proposal(block, latency)

        speed_factor = 1.0
        r = self.rng.random()
        if r < 0.15:
            speed_factor = 1.35     # learned policies overspeed sometimes
        elif r < 0.25:
            speed_factor = 0.8

        # sequential waypoint progression: never re-target an already-passed waypoint
        wps = self._waypoints(obs)
        while self.wp_idx < len(wps) - 1 and math.dist(obs.position, wps[self.wp_idx]) < 0.45:
            self.wp_idx += 1
        target = wps[self.wp_idx]

        # inspect sway: once on station, gentle micro-motion keeps the loop alive
        on_station = math.dist(obs.position, obs.goal) < 0.30

        deltas: List[List[float]] = []
        pos = list(obs.position)
        for k in range(CHUNK_STEPS):
            if on_station:
                sway = 0.03 * math.sin(2.0 * math.pi * (obs.t + k * CHUNK_DT) / 6.0)
                deltas.append([0.0, sway * 0.2, 0.0])
                continue
            remaining = [target[i] - pos[i] for i in range(3)]
            dist = math.sqrt(sum(x * x for x in remaining))
            if dist < 0.12:
                deltas.append([0.0, 0.0, 0.0])
                continue
            v = min(VMAX * speed_factor, 0.9 * dist)
            step = [remaining[i] / dist * v * CHUNK_DT for i in range(3)]
            step = [step[i] + self.rng.gauss(0.0, 0.012) for i in range(3)]
            # close-range inspection bias: lean toward the gauge for a better look
            # (learned policies crowd the target; the guard's standoff sphere answers)
            if obs.distance_to_target < 2.2:
                gd = [obs.position[i] - pos[i] for i in range(3)]
                gnorm = math.sqrt(sum(x * x for x in gd)) or 1e-9
                for i in range(3):
                    step[i] += 0.25 * gd[i] / gnorm * CHUNK_DT
            deltas.append(step)
            pos = [pos[i] + step[i] for i in range(3)]

        # orientation chunk: slew the camera toward the gauge from the *predicted*
        # position, anchored on the ACTUAL current attitude (the agent must not
        # assume it is already facing the target).
        d_orients: List[List[float]] = []
        pos = list(obs.position)
        yaw_now = obs.yaw
        pitch_now = obs.pitch
        for k in range(CHUNK_STEPS):
            pos = [pos[i] + deltas[k][i] for i in range(3)]
            yaw_t = self._yaw_to(pos, obs)
            pitch_t = self._pitch_to(pos, obs)
            d_orients.append([0.6 * (yaw_t - yaw_now), 0.6 * (pitch_t - pitch_now), 0.0])
            yaw_now += d_orients[-1][0]
            pitch_now += d_orients[-1][1]

        timeout_stress = 0.10 * min(obs.timeout_streak, 3)
        confidence = max(0.05, min(0.99, 0.52 + 0.45 * clarity - timeout_stress))
        stop = [0.03] * CHUNK_STEPS
        if clarity < 0.35:
            stop = [0.75] * CHUNK_STEPS

        block = ActionBlock(
            frame="body", horizon=CHUNK_STEPS, dt=CHUNK_DT,
            delta_position=self._to_body(deltas, obs.yaw),
            delta_orientation=d_orients,
            stop_probability=stop, confidence=confidence,
            agent=self.name, seq=self.chunk_seq,
        )
        self.chunk_seq += 1
        return Proposal(block, latency)

    @staticmethod
    def _to_body(deltas_world: List[List[float]], yaw: float) -> List[List[float]]:
        """The block declares frame='body', so world-plan deltas must be rotated
        into the body frame (Rz(-yaw)) before emission."""
        cy, sy = math.cos(yaw), math.sin(yaw)
        return [[d[0] * cy + d[1] * sy, -d[0] * sy + d[1] * cy, d[2]] for d in deltas_world]

    # ------------------------------------------------------------------ #
    @staticmethod
    def _yaw_to(pos: List[float], obs: Observation) -> float:
        """Yaw that points the camera boresight at the gauge from pos."""
        d = [obs.target[i] - pos[i] for i in range(2)]
        if math.hypot(*d) < 1e-6:
            return 0.0
        return math.atan2(d[1], d[0])

    @staticmethod
    def _pitch_to(pos: List[float], obs: Observation) -> float:
        """Camera pitch toward the gauge (positive = nose down)."""
        dx = math.hypot(obs.target[0] - pos[0], obs.target[1] - pos[1])
        dz = obs.target[2] - pos[2]
        return math.atan2(-dz, max(dx, 1e-6))
