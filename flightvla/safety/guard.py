"""SafetyGuard — validate / repair / reject agent action blocks before PX4 sees them.

Pipeline (every check is logged and rendered in the evaluation report):

  1. schema          — ActionBlock v0.1 well-formedness (PWM/rpm/MAVLink cannot even be expressed)
  2. frame           — body -> world transform using the current yaw
  3. perception-age  — reject chunks derived from stale camera frames
  4. stop-signal     — honour the agent's own stop_probability (instruction: hover when unsure)
  5. geofence        — hard fence box
  6. keep-out zones  — project the raw path out of the yellow cylinder (+margin), smooth
  7. standoff        — enforce the minimum distance sphere around the target (+margin)
  8. kinematics      — velocity / acceleration / jerk caps with momentum awareness
  9. feasibility     — can THIS airframe track the repaired path? else time-stretch, else reject
 10. allocation      — warn when the repair lands near control-allocation saturation
 11. camera          — annotate what the airframe physically cannot do (e.g. pitch a quad camera
                       while translating)

The guard never talks to motors: its output is a *setpoint stream* handed to the
offboard backend (PX4 in the field), which keeps owning EKF and all closed loops.
If the offboard stream itself dies, PX4's own failsafe takes over — that is the
execution boundary this runtime is built on.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from ..schema import ActionBlock, SchemaError, zeros_block
from ..vehicles.base import G, Vehicle

# guard margins — deliberately conservative, tunable per task
# guard margins — deliberately conservative, tunable per task.
# both must cover the vehicle's braking overshoot when the agent stream runs
# dry (timeout / hold): braking distance at vmax 1.6 m/s is ~0.35 m.
STANDOFF_MARGIN = 0.40      # metres beyond the instructed minimum distance
KEEPOUT_MARGIN = 0.55       # metres beyond the painted zone boundary (covers
                            # resample chord dip + braking overshoot + tracking error)
FENCE_MARGIN = 0.20
JERK_MAX = 12.0             # m/s^3
KIN_ACCEL_FRACTION = 0.8    # repaired paths keep 20% accel headroom for tracking
SATURATION_WARN = 0.92      # fraction of actuation budget
FEASIBILITY_REJECT_SCALE = 3.5
FRAME_MAX_AGE = 0.8         # seconds; older camera data -> reject
STOP_PROB_THRESHOLD = 0.6


@dataclass
class CheckResult:
    name: str
    status: str            # "pass" | "modified" | "reject" | "warn"
    detail: str = ""

    def to_dict(self) -> dict:
        return {"name": self.name, "status": self.status, "detail": self.detail}


@dataclass
class GuardResult:
    status: str                     # "accept" | "modify" | "reject" | "hold"
    checks: List[CheckResult] = field(default_factory=list)
    fallback: Optional[str] = None  # None | "hold" | "rtl"
    refs: List[List[float]] = field(default_factory=list)        # world-frame setpoints
    orient_refs: List[List[float]] = field(default_factory=list)  # [yaw, pitch] setpoints
    dts: List[float] = field(default_factory=list)                # per-step durations
    raw_path: List[List[float]] = field(default_factory=list)    # what the agent intended
    stats: dict = field(default_factory=dict)
    raw_block: dict = field(default_factory=dict)
    repaired_block: dict = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.status in ("accept", "modify", "hold")

    def to_log(self) -> dict:
        return {
            "status": self.status,
            "fallback": self.fallback,
            "checks": [c.to_dict() for c in self.checks],
            "stats": self.stats,
        }


def _norm(v: List[float]) -> float:
    return math.sqrt(sum(x * x for x in v))


class SafetyGuard:
    def __init__(self, scene, vehicle: Vehicle):
        self.scene = scene
        self.vehicle = vehicle

    # ================================================================== #
    def process(self, block: ActionBlock, t_arrival: float,
                frame_age: float) -> GuardResult:
        """Validate & repair one action block. `t_arrival` is when the block is
        ready to execute; `frame_age` is the age of the newest camera frame."""
        res = GuardResult(status="accept")
        res.raw_block = block.to_dict()

        # 1. schema ------------------------------------------------------
        try:
            block.validate()
            res.checks.append(CheckResult("schema", "pass",
                                          f"ActionBlock v0.1, {block.horizon} steps @ {block.dt}s"))
        except SchemaError as e:
            res.checks.append(CheckResult("schema", "reject", str(e)))
            res.status, res.fallback = "reject", "hold"
            return self._finalize(res, block)

        # 2. frame transform ---------------------------------------------
        yaw = self.vehicle.att.yaw
        cy, sy = math.cos(yaw), math.sin(yaw)
        if block.frame == "body":
            dw = [[d[0] * cy - d[1] * sy, d[0] * sy + d[1] * cy, d[2]]
                  for d in block.delta_position]
            res.checks.append(CheckResult("frame", "pass",
                                          f"body -> world (yaw {math.degrees(yaw):.0f} deg)"))
        else:
            dw = [list(d) for d in block.delta_position]
            res.checks.append(CheckResult("frame", "pass", "world"))

        p_now = list(self.vehicle.p)
        refs = []
        p = p_now
        for k in range(block.horizon):
            p = [p[i] + dw[k][i] for i in range(3)]
            refs.append(list(p))
        res.raw_path = [list(r) for r in refs]

        # orientation: cumulative absolute targets from the current attitude
        yaw0, pitch0 = self.vehicle.att.yaw, self.vehicle.att.pitch
        orients = []
        y, pr = yaw0, pitch0
        for k in range(block.horizon):
            dy, dp, _ = block.delta_orientation[k]
            y += dy
            pr += dp
            orients.append([y, pr])

        # 3. perception age ------------------------------------------------
        if frame_age > FRAME_MAX_AGE:
            res.checks.append(CheckResult(
                "perception-age", "reject",
                f"camera frame is {frame_age:.2f} s old (limit {FRAME_MAX_AGE:.1f} s)"))
            res.status, res.fallback = "reject", "hold"
            return self._finalize(res, block)
        res.checks.append(CheckResult("perception-age", "pass", f"frame age {frame_age:.2f} s"))

        # 4. agent stop signal ----------------------------------------------
        stop_k = block.first_stop_step(STOP_PROB_THRESHOLD)
        if stop_k is not None:
            # hover = hold POSITION, but the camera may keep slewing toward the
            # target (otherwise a degraded view could never recover)
            yaw0c, pitch0c = self.vehicle.att.yaw, self.vehicle.att.pitch
            oy, op = yaw0c, pitch0c
            n_hold = block.horizon
            rate_y_h = self.vehicle.limits.yaw_rate_max
            rate_p_h = 0.0 if self.vehicle.underactuated else self.vehicle.limits.pitch_rate_max
            orients_hold = []
            for k in range(n_hold):
                dy, dp, _ = block.delta_orientation[min(k, len(block.delta_orientation) - 1)]
                oy = _clamp_step(oy, oy + dy, rate_y_h * block.dt)
                op = _clamp_step(op, op + dp, rate_p_h * block.dt)
                orients_hold.append([oy, op])
            if self.vehicle.underactuated:
                for o in orients_hold:
                    o[1] = pitch0c
            res.orient_refs = orients_hold
            res.dts = [block.dt] * n_hold
            if stop_k == 0:
                res.checks.append(CheckResult(
                    "stop-signal", "reject",
                    f"stop_probability={block.stop_probability[0]:.2f} at step 0 "
                    f"-> hold position (camera keeps slewing)"))
                res.status, res.fallback = "reject", "hold"
                return self._finalize(res, block)
            refs = refs[:stop_k]
            orients = orients[:stop_k]
            res.checks.append(CheckResult(
                "stop-signal", "modified",
                f"truncated at step {stop_k} (stop_probability "
                f"{block.stop_probability[stop_k]:.2f} > {STOP_PROB_THRESHOLD}) -> hover after"))
            res.status = "hold"
            res.fallback = "hold"
        else:
            res.checks.append(CheckResult("stop-signal", "pass", "no stop signalled"))
        if not refs:
            return self._finalize(res, block)
        dt = block.dt
        dts = [dt] * len(refs)

        # 5. geofence ---------------------------------------------------------
        f_changed = any(self.scene.clamp_to_fence(r, FENCE_MARGIN) for r in refs)
        res.checks.append(CheckResult(
            "geofence", "modified" if f_changed else "pass",
            "clamped to fence" if f_changed else "inside fence"))

        # 6. keep-out zones: wall-follow repair. Radial projection is only valid
        # for shallow chords; for a deep cut we slide along the inflated circle
        # towards each target. Crossing chunks are re-emitted as a fine-grained
        # detour (more steps than the agent's horizon).
        depth, n_inside = self._keepout_stats(res.raw_path)
        refs, z_changed = self._avoid_keepout(refs, p_now)
        dts = [dt] * len(refs)
        while len(orients) < len(refs):        # extended detour: hold the last
            orients.append(list(orients[-1]))  # attitude along the added arc
        del orients[len(refs):]
        if n_inside:
            res.checks.append(CheckResult(
                "keep-out", "modified" if z_changed else "pass",
                f"{n_inside}/{len(res.raw_path)} raw points inside '{self.scene.keepout.label}'"
                f" (max depth {depth:.2f} m) -> repaired along the zone boundary"
                f" (+{KEEPOUT_MARGIN:.2f} m margin)"))
            if res.status == "accept":
                res.status = "modify"
        else:
            res.checks.append(CheckResult("keep-out", "pass", "path clears the zone"))

        # 7. standoff sphere --------------------------------------------------------
        dmin_raw = min(self.scene.distance_to_gauge(r) for r in res.raw_path)
        s_changed = self._enforce_standoff(refs)
        r_min = self.scene.constraints_min_distance() + STANDOFF_MARGIN
        if s_changed:
            res.checks.append(CheckResult(
                "standoff", "modified",
                f"raw min distance {dmin_raw:.2f} m < {r_min:.2f} m -> pushed out"
                f" (instruction: >= {self.scene.constraints_min_distance()} m)"))
            if res.status == "accept":
                res.status = "modify"
        else:
            res.checks.append(CheckResult(
                "standoff", "pass", f"min distance {dmin_raw:.2f} m >= {r_min:.2f} m"))

        # 8. kinematics (momentum aware: the vehicle keeps flying between chunks) ----
        kin_modified = self._kinematic_pass(refs, p_now, dt)
        # caps may bend the tail back towards the zone: re-project the last points
        for r in refs[-2:]:
            self.scene.project_out_of_keepout(r, KEEPOUT_MARGIN)
            self._enforce_standoff([r])
        res.checks.append(CheckResult(
            "kinematics", "modified" if kin_modified else "pass",
            f"v/a/jerk caps{' applied' if kin_modified else ' satisfied'}"
            f" (v_max {self.vehicle.limits.vmax:.1f} m/s)"))

        # 9. vehicle feasibility -------------------------------------------------
        scale, a_needed = self._feasibility_scale(refs, p_now, dt)
        if scale > 1.0 + 1e-6:
            if scale > FEASIBILITY_REJECT_SCALE:
                res.checks.append(CheckResult(
                    "feasibility", "reject",
                    f"path needs {a_needed:.1f} m/s^2, {self.vehicle.name} can deliver "
                    f"{self.vehicle.horizontal_accel_max():.1f} -> reject, hold and re-plan"))
                res.status, res.fallback = "reject", "hold"
                return self._finalize(res, block)
            for k, r in enumerate(refs):
                for i in range(3):
                    r[i] = p_now[i] + (r[i] - p_now[i]) / scale
                dts[k] = dts[k] * scale
            res.checks.append(CheckResult(
                "feasibility", "modified",
                f"time-stretched x{scale:.2f}: {self.vehicle.name} amax "
                f"{self.vehicle.horizontal_accel_max():.1f} < needed {a_needed:.1f} m/s^2"))
            if res.status == "accept":
                res.status = "modify"
        else:
            res.checks.append(CheckResult(
                "feasibility", "pass",
                f"needs {a_needed:.1f} of {self.vehicle.horizontal_accel_max():.1f} m/s^2"))

        # 10. allocation margin -----------------------------------------------------
        frac = a_needed / max(1e-6, self.vehicle.horizontal_accel_max())
        if frac > SATURATION_WARN:
            res.checks.append(CheckResult(
                "allocation", "warn",
                f"accel demand at {min(1.0, frac) * 100:.0f}% of actuation budget"))
        else:
            res.checks.append(CheckResult(
                "allocation", "pass", f"{frac * 100:.0f}% of actuation budget"))

        # 11. camera feasibility (annotation) ----------------------------------------
        if self.vehicle.underactuated:
            pitch_span = max(o[1] for o in orients) - min(o[1] for o in orients)
            if pitch_span > math.radians(5.0):
                res.checks.append(CheckResult(
                    "camera", "warn",
                    "quad camera pitch follows body tilt: pitch commands while translating "
                    "will not be honoured (airframe is underactuated)"))
            else:
                res.checks.append(CheckResult("camera", "pass", "yaw-only camera command"))
        else:
            res.checks.append(CheckResult(
                "camera", "pass",
                "fully-actuated: yaw+pitch commandable independently of translation"))

        # orientation rate limits ------------------------------------------------------
        rate_y = self.vehicle.limits.yaw_rate_max
        rate_p = 0.0 if self.vehicle.underactuated else self.vehicle.limits.pitch_rate_max
        oy, op = yaw0, pitch0
        for k, o in enumerate(orients):
            oy = o[0] = _clamp_step(oy, o[0], rate_y * dts[k])
            op = o[1] = _clamp_step(op, o[1], rate_p * dts[k])
        if self.vehicle.underactuated:
            for o in orients:
                o[1] = pitch0   # quad camera pitch is physics-owned, not command-owned

        res.refs = refs
        res.orient_refs = orients
        res.dts = dts
        return self._finalize(res, block)

    # ================================================================== #
    def _enforce_standoff(self, refs: List[List[float]]) -> bool:
        changed = False
        r_min = self.scene.constraints_min_distance() + STANDOFF_MARGIN
        for r in refs:
            d = self.scene.distance_to_gauge(r)
            if d < r_min - 1e-9:
                if d < 1e-6:
                    r[0] = self.scene.gauge[0] - r_min   # degenerate: push along -x
                else:
                    f = r_min / d
                    for i in range(3):
                        r[i] = self.scene.gauge[i] + (r[i] - self.scene.gauge[i]) * f
                changed = True
        return changed

    def _avoid_keepout(self, refs: List[List[float]],
                       p_now: List[float]) -> Tuple[List[List[float]], bool]:
        """Wall-follow repair: walk the raw path; wherever a segment would enter
        the inflated keep-out circle, slide along the boundary (shortest angular
        direction to the target) until the direct segment clears, then resume.
        Crossing chunks come back as a fine-grained boundary-following path that
        may be longer than the original horizon."""
        k = self.scene.keepout
        R = k.r + KEEPOUT_MARGIN
        cx, cy = k.cx, k.cy

        def seg_clear(a, b) -> bool:
            dx, dy = b[0] - a[0], b[1] - a[1]
            L2 = dx * dx + dy * dy
            if L2 < 1e-12:
                return math.hypot(a[0] - cx, a[1] - cy) >= R
            t = max(0.0, min(1.0, ((cx - a[0]) * dx + (cy - a[1]) * dy) / L2))
            return math.hypot(a[0] + t * dx - cx, a[1] + t * dy - cy) >= R - 1e-9

        out: List[List[float]] = []
        q = [p_now[0], p_now[1]]
        changed = False
        for tgt in refs:
            tx, ty = tgt[0], tgt[1]
            # a target buried inside the inflated zone can never be reached:
            # clamp it to the boundary (angle preserved) before sliding towards it,
            # otherwise the slide oscillates around its angle.
            dt_o = math.hypot(tx - cx, ty - cy)
            if dt_o < R:
                if dt_o < 1e-9:
                    a_q0 = math.atan2(q[1] - cy, q[0] - cx)
                    tx, ty = cx + R * math.cos(a_q0), cy + R * math.sin(a_q0)
                else:
                    tx, ty = cx + (tx - cx) * R / dt_o, cy + (ty - cy) * R / dt_o
                changed = True
            dq = math.hypot(q[0] - cx, q[1] - cy)
            if dq < R - 1e-9:   # started inside the margin: step out radially
                if dq < 1e-9:
                    q = [cx + R, cy]
                else:
                    q = [cx + (q[0] - cx) * R / dq, cy + (q[1] - cy) * R / dq]
                changed = True
            guard = 0
            while not seg_clear(q, (tx, ty)) and guard < 64:
                a_q = math.atan2(q[1] - cy, q[0] - cx)
                a_t = math.atan2(ty - cy, tx - cx)
                da = (a_t - a_q + math.pi) % (2.0 * math.pi) - math.pi
                d_ang = min(abs(da), math.hypot(tx - q[0], ty - q[1]) / R)
                a_n = a_q + (d_ang if da >= 0 else -d_ang)
                nq = [cx + R * math.cos(a_n), cy + R * math.sin(a_n)]
                if math.hypot(nq[0] - q[0], nq[1] - q[1]) < 1e-9:
                    break
                out.append([nq[0], nq[1], tgt[2]])
                q = nq
                changed = True
                guard += 1
            out.append([tx, ty, tgt[2]])
            q = [tx, ty]
        # resample: a crossed zone yields a fine-grained boundary-following path
        # (short steps -> the vehicle automatically slows down near the obstacle,
        # and short chords cannot dip back inside the circle). Non-crossing
        # chunks keep the original horizon timing untouched.
        if changed and len(out) >= 1:
            ds = 0.8 * self.vehicle.limits.vmax * self.scene.chunk_dt
            poly = [[p_now[0], p_now[1], p_now[2]]] + out
            cum = [0.0]
            for i in range(1, len(poly)):
                cum.append(cum[-1] + math.dist(poly[i - 1], poly[i]))
            total = cum[-1]
            if total > 1e-9:
                n_steps = max(len(out), int(math.ceil(total / ds)))
                resampled, j = [], 0
                for m in range(1, n_steps + 1):
                    s = total * m / n_steps
                    while j < len(cum) - 2 and cum[j + 1] < s:
                        j += 1
                    seg_len = (cum[j + 1] - cum[j]) or 1e-9
                    f = (s - cum[j]) / seg_len
                    resampled.append([
                        poly[j][i] + (poly[j + 1][i] - poly[j][i]) * f
                        for i in range(3)])
                out = resampled
        new_refs = [list(o) for o in out]
        for r in new_refs:
            if self.scene.project_out_of_keepout(r, KEEPOUT_MARGIN):
                changed = True
        return new_refs, changed

    def _kinematic_pass(self, refs: List[List[float]], p_now: List[float],
                        dt: float) -> bool:
        """Clamp per-step displacement to v/a/jerk limits, starting from the
        vehicle's CURRENT velocity (chunks hand over mid-flight)."""
        lim = self.vehicle.limits
        vmax = lim.vmax
        amax = self.vehicle.horizontal_accel_max() * KIN_ACCEL_FRACTION
        v_now = self.vehicle.v
        d_prev = [v_now[i] * dt for i in range(3)]
        d_prev2 = [v_now[i] * dt for i in range(3)]
        modified = False
        for k in range(len(refs)):
            p_prev = refs[k - 1] if k else p_now
            d = [refs[k][i] - p_prev[i] for i in range(3)]
            # velocity cap
            n = _norm(d)
            if n > vmax * dt + 1e-9:
                d = [x * (vmax * dt / n) for x in d]
                modified = True
            # acceleration cap (change of per-step displacement)
            dd = [d[i] - d_prev[i] for i in range(3)]
            ndd = _norm(dd)
            if ndd > amax * dt * dt + 1e-9:
                s = amax * dt * dt / ndd
                d = [d_prev[i] + dd[i] * s for i in range(3)]
                modified = True
            # jerk cap
            ddd = [(d[i] - d_prev[i]) - (d_prev[i] - d_prev2[i]) for i in range(3)]
            j_lim = JERK_MAX * dt * dt * dt
            nddd = _norm(ddd)
            if nddd > j_lim + 1e-9:
                s = j_lim / nddd
                d = [d_prev[i] + (d_prev[i] - d_prev2[i]) + ddd[i] * s for i in range(3)]
                modified = True
            refs[k] = [p_prev[i] + d[i] for i in range(3)]
            d_prev2 = d_prev
            d_prev = d
        return modified

    def plan_rtl(self) -> GuardResult:
        """Guard-planned Return To Launch: fly home at cruise (constraints still
        enforced), descend and land. Used when the agent stream dies."""
        res = GuardResult(status="accept")
        res.checks.append(CheckResult("fallback", "modified",
                                      "agent stream lost -> guard-planned RTL"))
        home = self.scene.home
        p = list(self.vehicle.p)
        cruise = max(1.6, p[2])
        dt = self.scene.chunk_dt
        vmax = self.vehicle.limits.vmax
        refs: List[List[float]] = []
        target = [home[0], home[1], cruise]
        n = _norm([target[i] - p[i] for i in range(3)])
        steps = max(2, int(n / (vmax * dt)))
        for k in range(1, steps + 1):
            q = [p[i] + (target[i] - p[i]) * k / steps for i in range(3)]
            self.scene.project_out_of_keepout(q, KEEPOUT_MARGIN)
            self.scene.clamp_to_fence(q, FENCE_MARGIN)
            refs.append(q)
        for z in [x / 20.0 for x in range(int(cruise * 20), 2, -1)]:
            refs.append([home[0], home[1], max(0.10, z)])
        refs.append([home[0], home[1], 0.10])
        orients = [[self.vehicle.att.yaw, 0.0] for _ in refs]
        res.refs = refs
        res.orient_refs = orients
        res.dts = [dt] * len(refs)
        res.raw_path = [list(r) for r in refs]
        res.fallback = "rtl"
        dmin = min(self.scene.distance_to_gauge(r) for r in refs)
        res.stats = {"min_dist_gauge_repaired": round(dmin, 3)}
        _, n_inside = self._keepout_stats(refs)
        res.checks.append(CheckResult(
            "keep-out", "pass" if not n_inside else "modified",
            "RTL path re-checked against zones"))
        return res

    # ------------------------------------------------------------------ #
    def _keepout_stats(self, path: List[List[float]]) -> Tuple[float, int]:
        depth, n = 0.0, 0
        for r in path:
            d = self.scene.keepout.horizontal_distance(r)
            if self.scene.keepout.z0 <= r[2] <= self.scene.keepout.z1 and d < self.scene.keepout.r:
                n += 1
                depth = max(depth, self.scene.keepout.r - d)
        return depth, n

    def _feasibility_scale(self, refs: List[List[float]], p_now: List[float],
                           dt: float) -> Tuple[float, float]:
        """Return (time-stretch factor, max horizontal accel needed)."""
        prev_v = list(self.vehicle.v)
        a_max_needed = 0.0
        for k, r in enumerate(refs):
            p_prev = refs[k - 1] if k else p_now
            d = [r[i] - p_prev[i] for i in range(3)]
            v_k = [d[i] / dt for i in range(3)]
            a_k = [(v_k[i] - prev_v[i]) / dt for i in range(3)]
            a_max_needed = max(a_max_needed, math.hypot(a_k[0], a_k[1]))
            prev_v = v_k
        allowed = self.vehicle.horizontal_accel_max()
        if a_max_needed > allowed + 1e-6:
            return a_max_needed / allowed, a_max_needed
        return 1.0, a_max_needed

    # ------------------------------------------------------------------ #
    def _finalize(self, res: GuardResult, block: ActionBlock) -> GuardResult:
        dmin_rep = min((self.scene.distance_to_gauge(r) for r in (res.refs or [list(self.vehicle.p)])),
                       default=self.scene.distance_to_gauge(self.vehicle.p))
        _, n_inside = self._keepout_stats(res.refs) if res.refs else (0.0, 0)
        depth_raw, n_raw = self._keepout_stats(res.raw_path) if res.raw_path else (0.0, 0)
        dmin_raw = min((self.scene.distance_to_gauge(r) for r in res.raw_path), default=None)
        res.stats = {
            "min_dist_gauge_raw": None if dmin_raw is None else round(dmin_raw, 3),
            "min_dist_gauge_repaired": round(dmin_rep, 3),
            "keepout_raw_points": n_raw,
            "keepout_raw_depth_m": round(depth_raw, 3),
            "keepout_repaired_points": n_inside,
            "confidence": round(block.confidence, 3),
        }
        res.repaired_block = {
            "frame": "world", "horizon": len(res.refs),
            "dt": round(res.dts[0], 3) if res.dts else None,
            "position_refs": [[round(x, 3) for x in r] for r in res.refs],
            "orientation_refs": [[round(o[0], 3), round(o[1], 3)] for o in res.orient_refs],
        }
        return res


def _clamp_step(current: float, target: float, max_step: float) -> float:
    if max_step <= 0:
        return current
    return current + max(-max_step, min(max_step, target - current))
