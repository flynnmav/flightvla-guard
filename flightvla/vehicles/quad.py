"""Classic underactuated quadrotor.

Horizontal acceleration requires body tilt; a body-fixed camera tilts with the
airframe. Camera pitch therefore follows translational accelerations and CANNOT
follow an agent's independent pitch command — the exact physical reason a
quadrotor loses framing of a target while translating laterally/fore-aft.
"""
from __future__ import annotations

import math
from typing import List

from .base import G, Limits, Vehicle, _rate_track


class QuadVehicle(Vehicle):
    kind = "underactuated"

    def __init__(self, name: str = "quad"):
        super().__init__(name, Limits(
            vmax=1.6, amax_h=5.0, amax_v=3.0,
            tilt_max=math.radians(35.0),
            yaw_rate_max=math.radians(90.0),
            pitch_rate_max=0.0,   # pitch is decided by physics, not commanded
        ), camera_mount_deg=12.0)

    def horizontal_accel_max(self) -> float:
        # tilt-limited: a_max = g * tan(tilt_max)
        return min(self.limits.amax_h, G * math.tan(self.limits.tilt_max))

    def _update_attitude(self, a_used: List[float], dt: float) -> None:
        # body-frame horizontal accel (yaw rotates the frame)
        cy, sy = math.cos(self.att.yaw), math.sin(self.att.yaw)
        a_bx = a_used[0] * cy + a_used[1] * sy     # forward
        a_by = -a_used[0] * sy + a_used[1] * cy    # left
        pitch_target = math.atan2(a_bx, G)         # fwd accel -> nose down -> camera pitches down
        roll_target = math.atan2(a_by, G)          # left accel -> bank left (horizon rolls)
        self.att.pitch = _rate_track(self.att.pitch, pitch_target,
                                     math.radians(240.0), dt, tau=self.tilt_tau)
        self.att.roll = _rate_track(self.att.roll, roll_target,
                                    math.radians(240.0), dt, tau=self.tilt_tau)
        # yaw is the only independently commanded camera axis
        self.att.yaw = _rate_track(self.att.yaw, self._yaw_cmd,
                                   self.limits.yaw_rate_max, dt)

    # the autopilot hands us the commanded yaw each tick via `set_orientation_cmd`
    _yaw_cmd = 0.0

    def set_orientation_cmd(self, yaw_cmd: float, pitch_cmd: float) -> None:
        self._yaw_cmd = yaw_cmd
        # pitch command is deliberately ignored: an underactuated airframe cannot
        # hold camera pitch independent of translational accel. Guard knows this.
