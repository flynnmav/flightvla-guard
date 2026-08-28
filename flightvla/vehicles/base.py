"""Vehicle models: double-integrator translational dynamics with physically honest
attitude semantics.

These are *not* meant to replace PX4. They are the minimal stand-in for the
PX4 cascade (position -> velocity -> acceleration -> attitude -> rate loops)
used by the built-in simulator and evaluation platform, so that the guard has a
realistic feasibility question to answer:

  "can THIS airframe actually fly the proposed 6-DoF motion?"

  - underactuated quad: to accelerate horizontally the body must tilt, and a
    body-fixed camera tilts with it. Camera pitch is NOT independently
    controllable. This is where VLA plans start hurting.
  - fully-actuated omni (tilted-rotor hex/octo): horizontal force is produced
    without tilting, so position and attitude are decoupled: the vehicle can
    translate while keeping the camera locked on a target.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

G = 9.81


@dataclass
class Limits:
    vmax: float            # m/s, all axes (kept moderate so the guard's static
                         # margins can cover the braking distance at cruise speed)
    amax_h: float          # m/s^2 horizontal
    amax_v: float          # m/s^2 vertical
    tilt_max: float        # rad, maximum body tilt (hard for quad)
    yaw_rate_max: float    # rad/s
    pitch_rate_max: float  # rad/s (only meaningful for fully-actuated frames)
    camera_hfov: float = math.radians(90.0)


@dataclass
class AttitudeState:
    yaw: float = 0.0       # rad
    pitch: float = 0.0     # rad, positive = nose down (camera boresight pitches down)
    roll: float = 0.0      # rad, frame roll (horizon tilt), does not move a forward boresight

    def as_list(self) -> List[float]:
        return [self.yaw, self.pitch, self.roll]


class Vehicle:
    """Base class. Subclasses implement `apply_accel` attitude semantics."""

    kind = "abstract"

    def __init__(self, name: str, limits: Limits, tilt_tau: float = 0.25,
                 camera_mount_deg: float = 12.0):
        self.name = name
        self.limits = limits
        self.tilt_tau = tilt_tau   # first-order attitude lag, seconds
        # fixed camera mount: inspection rigs hang the camera at a downtilt angle.
        # this makes the framing problem real: ANY body pitch/roll now swings the
        # boresight, and only a fully-actuated airframe can hold the camera on
        # target while translating.
        self.camera_mount = math.radians(camera_mount_deg)
        self.p = [0.0, 0.0, 0.0]
        self.v = [0.0, 0.0, 0.0]
        self.a = [0.0, 0.0, 0.0]   # accel actually applied, world frame (incl. disturbance)
        self.att = AttitudeState()
        self.disturbance = [0.0, 0.0, 0.0]
        self.saturation = 0.0      # max |a_cmd| / amax over the last step (allocation margin proxy)

    # ------------------------------------------------------------------ #
    def reset(self, p: List[float], yaw: float) -> None:
        self.p = list(p)
        self.v = [0.0, 0.0, 0.0]
        self.a = [0.0, 0.0, 0.0]
        self.att = AttitudeState(yaw=yaw)
        self.disturbance = [0.0, 0.0, 0.0]
        self.saturation = 0.0

    @property
    def underactuated(self) -> bool:
        return self.kind == "underactuated"

    # ------------------------------------------------------------------ #
    def clip_accel(self, a_cmd: List[float]) -> List[float]:
        lim = self.limits
        out = [0.0, 0.0, 0.0]
        a_h = math.hypot(a_cmd[0], a_cmd[1])
        a_h_max = self.horizontal_accel_max()
        if a_h > a_h_max and a_h > 1e-9:
            s = a_h_max / a_h
            out[0], out[1] = a_cmd[0] * s, a_cmd[1] * s
        else:
            out[0], out[1] = a_cmd[0], a_cmd[1]
        out[2] = max(-lim.amax_v, min(lim.amax_v, a_cmd[2]))
        self.saturation = max(
            a_h / a_h_max if a_h_max > 0 else 0.0,
            abs(out[2]) / lim.amax_v if lim.amax_v > 0 else 0.0)
        return out

    def horizontal_accel_max(self) -> float:
        return self.limits.amax_h

    def step(self, a_cmd: List[float], dt: float) -> None:
        a = self.clip_accel(a_cmd)
        for i in range(3):
            a[i] += self.disturbance[i]
            self.v[i] += a[i] * dt
            self.p[i] += self.v[i] * dt
        self.a = a
        self._update_attitude(a, dt)

    # ------------------------------------------------------------------ #
    def _update_attitude(self, a_used: List[float], dt: float) -> None:
        raise NotImplementedError

    def boresight(self) -> List[float]:
        """Camera boresight unit vector in world frame.

        Body attitude (yaw, pitch, roll) composed with the fixed mount downtilt:
        pitch and mount simply add for the boresight elevation; roll swings the
        boresight laterally by sin(pitch+mount) — the physical reason a tilted
        quad moves the target off-frame while a level omni does not.
        """
        cm = math.cos(self.att.pitch + self.camera_mount)
        sm = math.sin(self.att.pitch + self.camera_mount)
        cr = math.cos(self.att.roll)
        sr = math.sin(self.att.roll)
        bx = cm
        by = sm * sr
        bz = -sm * cr
        yaw = self.att.yaw
        return [bx * math.cos(yaw) - by * math.sin(yaw),
                bx * math.sin(yaw) + by * math.cos(yaw),
                bz]

    def horizon_right(self) -> List[float]:
        yaw = self.att.yaw
        return [-math.sin(yaw), math.cos(yaw), 0.0]

    def bore_error(self, target: List[float]) -> Tuple[float, float, float]:
        """Total angle, in-frame horizontal offset and vertical offset (all rad)
        between the boresight and the direction to `target`."""
        d = [target[i] - self.p[i] for i in range(3)]
        n = math.sqrt(sum(x * x for x in d)) or 1e-9
        d = [x / n for x in d]
        b = self.boresight()
        dot = max(-1.0, min(1.0, sum(b[i] * d[i] for i in range(3))))
        total = math.acos(dot)
        # decompose into camera-plane offsets
        r = self.horizon_right()
        up = [b[1] * r[2] - b[2] * r[1], b[2] * r[0] - b[0] * r[2],
              b[0] * r[1] - b[1] * r[0]]
        horiz = math.atan2(sum(r[i] * d[i] for i in range(3)),
                           max(1e-6, sum(b[i] * d[i] for i in range(3))))
        vert = math.atan2(sum(up[i] * d[i] for i in range(3)),
                          max(1e-6, sum(b[i] * d[i] for i in range(3))))
        return total, horiz, vert

    # ------------------------------------------------------------------ #
    def required_tilt(self, a_h: float) -> float:
        return math.atan2(a_h, G)

    def describe(self) -> dict:
        return {
            "name": self.name, "kind": self.kind,
            "vmax": self.limits.vmax, "amax_h": self.limits.amax_h,
            "amax_v": self.limits.amax_v,
            "tilt_max_deg": round(math.degrees(self.limits.tilt_max), 1),
            "yaw_rate_max_deg": round(math.degrees(self.limits.yaw_rate_max), 1),
            "pitch_rate_max_deg": round(math.degrees(self.limits.pitch_rate_max), 1),
        }


def _rate_track(current: float, target: float, rate_max: float, dt: float,
                tau: Optional[float] = None) -> float:
    """Move `current` toward `target`, rate-limited, optionally first-order lagged."""
    step = max(-rate_max * dt, min(rate_max * dt, target - current))
    out = current + step
    if tau is not None and tau > 1e-6:
        out = current + (out - current) * min(1.0, dt / tau)
    return out
