"""Fully-actuated omnidirectional multirotor (tilted-rotor hex / octo).

Produces horizontal force without tilting, so translation and attitude are
decoupled: the vehicle can strafe, orbit and resist gusts while the body-fixed
camera stays locked on the target. Both yaw and pitch are independently
commandable at hover. The price is lower side-force authority than a tilted
quad (smaller horizontal accel budget) — a trade-off the evaluation platform
makes visible instead of hiding.
"""
from __future__ import annotations

import math
from typing import List

from .base import Limits, Vehicle, _rate_track


class OmniVehicle(Vehicle):
    kind = "fully_actuated"

    def __init__(self, name: str = "omni-hex", rotors: int = 6):
        self.rotors = rotors
        super().__init__(name, Limits(
            vmax=1.6, amax_h=4.0, amax_v=3.0,
            tilt_max=math.radians(20.0),   # attitude excursions stay small by design
            yaw_rate_max=math.radians(180.0),
            pitch_rate_max=math.radians(90.0),
        ), camera_mount_deg=12.0)

    _yaw_cmd = 0.0
    _pitch_cmd = 0.0

    def _update_attitude(self, a_used: List[float], dt: float) -> None:
        # attitude is commanded, not forced by translation. `_pitch_cmd` is the
        # desired CAMERA pitch in the world frame; holding it requires pitching
        # the BODY against the mount angle — a fully-actuated airframe can do
        # that at a hover, an underactuated one cannot.
        self.att.yaw = _rate_track(self.att.yaw, self._yaw_cmd,
                                   self.limits.yaw_rate_max, dt)
        self.att.pitch = _rate_track(self.att.pitch, self._pitch_cmd - self.camera_mount,
                                     self.limits.pitch_rate_max, dt)
        self.att.roll = _rate_track(self.att.roll, 0.0,
                                    self.limits.pitch_rate_max, dt)

    def set_orientation_cmd(self, yaw_cmd: float, pitch_cmd: float) -> None:
        self._yaw_cmd = yaw_cmd
        self._pitch_cmd = pitch_cmd
