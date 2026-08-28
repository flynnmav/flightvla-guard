"""Closed-loop simulation: agent -> SafetyGuard -> offboard stream -> vehicle.

Clock: fixed 50 Hz physics ticks. Agent runs at the chunk period
(horizon x chunk_dt = 1.6 s for the default task); its (simulated) inference
latency determines when the chunk actually reaches the guard. Whatever the
guard approves becomes a world-frame setpoint stream — the stand-in for the
PX4 offboard topic — which the simplified PX4 cascade (position/velocity loops)
tracks. Faults (gust accel, visual loss, latency bias, offboard loss) are
evaluated every tick.

Everything needed by the HTML evaluation report is recorded: the agent's raw
intended path, the repaired path, per-check verdicts, events, and the full
state timeline.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Deque, List, Optional, Tuple

from .agents.base import Observation
from .agents import create_agent
from .faults import FaultSchedule
from .safety import SafetyGuard
from .schema import zeros_block
from .vehicles import create_vehicle
from . import __version__

PHYS_DT = 0.05
GUARD_OVERHEAD = 0.01   # fixed guard processing cost, seconds


class _RefQueue:
    """Time-ordered setpoint stream (the 'offboard topic'). New plans supersede
    pending ones: whatever a fresh approved chunk does not cover is dropped."""

    def __init__(self) -> None:
        self.q: Deque[Tuple[float, List[float], float, float]] = deque()

    def push(self, t_apply: float, p: List[float], yaw: float, pitch: float) -> None:
        self.q.append((t_apply, list(p), yaw, pitch))

    def invalidate_from(self, t: float) -> None:
        self.q = deque(item for item in self.q if item[0] < t)

    def next_at(self) -> Optional[float]:
        return self.q[0][0] if self.q else None

    def pop_due(self, t: float) -> Optional[Tuple[float, List[float], float, float]]:
        item = None
        while self.q and self.q[0][0] <= t + 1e-9:
            item = self.q.popleft()
        return item


class Runner:
    def __init__(self, agent_name: str, vehicle_name: str, scene,
                 faults: FaultSchedule, seed: int = 7,
                 duration: Optional[float] = None):
        self.scene = scene
        self.faults = faults
        self.seed = seed
        self.vehicle = create_vehicle(vehicle_name)
        self.agent = create_agent(agent_name)
        self.guard = SafetyGuard(scene, self.vehicle)
        self.duration = duration if duration is not None else scene.duration
        self.chunk_period = scene.chunk_horizon * scene.chunk_dt

    # ------------------------------------------------------------------ #
    def run(self) -> dict:
        import random
        rng = random.Random(self.seed)
        scene, veh = self.scene, self.vehicle

        veh.reset(list(scene.home), yaw=self._yaw_to(scene.home, scene.gauge))
        self.agent.reset(self.seed)

        queue = _RefQueue()
        ref = list(scene.home)
        yaw_cmd = veh.att.yaw
        pitch_cmd = veh.att.pitch

        timeline = {k: [] for k in (
            "t", "px", "py", "pz", "vx", "vy", "vz", "rx", "ry", "rz",
            "yaw", "pitch", "roll", "bore", "bore_h", "bore_v", "dist",
            "speed", "track_err", "energy", "wind", "hold", "on_target")}
        chunks: List[dict] = []
        events: List[dict] = []
        keyframes: List[dict] = []
        key_times = [3.5, 7.5, 13.8, 15.2, 19.8, 26.0]

        t = 0.0
        next_call = 0.0
        timeout_streak = 0
        energy = 0.0
        success_t: Optional[float] = None
        on_target_since: Optional[float] = None
        holding = False
        hold_started: Optional[float] = None
        rtl_mode = False
        rtl_triggered = False
        end_t = self.duration
        last_frame_t = 0.0

        def event(kind: str, text: str) -> None:
            events.append({"t": round(t, 3), "kind": kind, "text": text})

        event("start", f"run starts: agent={self.agent.name}, vehicle={self.vehicle.name}")

        while t <= end_t + 1e-9:
            fault = self.faults.evaluate(t)
            veh.disturbance = list(fault.gust_accel)
            last_frame_t = t  # the camera streams every tick; visual loss hides the target

            # ---- offboard loss -> guard-planned RTL ---------------------
            if fault.offboard_loss and not rtl_mode:
                rtl_mode = True
                rtl_triggered = True
                res = self.guard.plan_rtl()
                at = t
                for k, (p, o, d) in enumerate(zip(res.refs, res.orient_refs, res.dts)):
                    queue.push(at + k * d, p, o[0], o[1])
                event("rtl", "agent stream lost -> guard plans RTL (constraints re-checked)")
                chunks.append({
                    "t_call": round(t, 3), "t_arrival": round(t, 3), "latency": 0.0,
                    "timeout": False, "confidence": 1.0,
                    "status": res.status, "checks": [c.to_dict() for c in res.checks],
                    "stats": res.stats, "fallback": "rtl",
                    "raw_path": [], "repaired_path": res.refs[::4],
                    "raw_block": {"agent": "guard", "note": "internal RTL plan"},
                    "repaired_block": {},
                })

            # ---- agent call at chunk boundaries --------------------------
            if not rtl_mode and t >= next_call - 1e-9:
                bore, bore_h, bore_v = veh.bore_error(scene.gauge)
                dist = scene.distance_to_gauge(veh.p)
                clarity = self._clarity(bore, dist, fault.visual_loss)
                obs = Observation(
                    t=t, position=list(veh.p), velocity=list(veh.v),
                    yaw=veh.att.yaw, pitch=veh.att.pitch,
                    bore_error_deg=math.degrees(bore),
                    bore_horiz_deg=math.degrees(bore_h),
                    bore_vert_deg=math.degrees(bore_v),
                    distance_to_target=dist, target=scene.gauge,
                    target_visible=not fault.visual_loss, clarity=clarity,
                    timeout_streak=timeout_streak, goal=list(scene.standoff))
                prop = self.agent.propose(obs)
                latency = max(0.02, prop.latency + fault.latency_bias)
                t_arrival = next_call + latency + GUARD_OVERHEAD
                timed_out = latency > scene.agent_budget
                if timed_out:
                    timeout_streak += 1
                    event("timeout", f"agent inference {latency * 1000:.0f} ms "
                                     f"> budget {scene.agent_budget * 1000:.0f} ms (streak {timeout_streak})")
                else:
                    timeout_streak = 0

                frame_age = t - last_frame_t
                res = self.guard.process(prop.block, t_arrival, frame_age)
                at = t_arrival
                queue.invalidate_from(at)   # the fresh plan supersedes pending refs
                n = max(len(res.refs), len(res.orient_refs))
                for k in range(n):
                    p = res.refs[k] if k < len(res.refs) else list(veh.p)
                    o = res.orient_refs[k] if k < len(res.orient_refs) \
                        else (veh.att.yaw, veh.att.pitch)
                    d = res.dts[k] if k < len(res.dts) else scene.chunk_dt
                    queue.push(at + k * d, p, o[0], o[1])
                if res.fallback == "hold" and not res.refs:
                    self._push_hold(queue, at + n * scene.chunk_dt, list(veh.p),
                                    veh.att.yaw, veh.att.pitch,
                                    next_call + scene.agent_budget + 0.2)
                    if not holding:
                        holding, hold_started = True, t
                        event("hold", "hold triggered: " + "; ".join(
                            c.detail for c in res.checks if c.status in ("reject", "modified"))[:140])
                elif res.fallback == "hold" and res.refs:
                    self._push_hold(queue, at + len(res.refs) * scene.chunk_dt,
                                    list(veh.p), veh.att.yaw, veh.att.pitch,
                                    next_call + scene.agent_budget + 0.2)
                    if not holding:
                        holding, hold_started = True, t
                        event("hold", "hold triggered: " + "; ".join(
                            c.detail for c in res.checks if c.status in ("reject", "modified"))[:140])
                elif holding:
                    holding = False
                    event("hold-end", f"held {t - hold_started:.1f} s, agent stream resumed")

                chunks.append({
                    "t_call": round(next_call, 3), "t_arrival": round(at, 3),
                    "latency": round(latency, 3), "timeout": timed_out,
                    "confidence": round(prop.block.confidence, 3),
                    "status": res.status, "checks": [c.to_dict() for c in res.checks],
                    "stats": res.stats, "fallback": res.fallback,
                    "raw_path": [[round(x, 3) for x in r] for r in res.raw_path],
                    "repaired_path": [[round(x, 3) for x in r] for r in res.refs],
                    "raw_block": res.raw_block, "repaired_block": res.repaired_block,
                })
                next_call += self.chunk_period

            # ---- reference from the offboard stream ----------------------
            item = queue.pop_due(t)
            if item is not None:
                _, ref, yaw_cmd, pitch_cmd = item
            # when the stream drains the last reference persists (position hold)

            # ---- autopilot: simplified PX4 position/velocity cascade ------
            # stiff gains on purpose: PX4's position loop is tight, and a soft
            # loop would let wind push the vehicle far off the guarded reference
            a_cmd = [0.0, 0.0, 0.0]
            for i in range(3):
                v_des = 2.4 * (ref[i] - veh.p[i])
                v_des = max(-veh.limits.vmax, min(veh.limits.vmax, v_des))
                a_cmd[i] = 3.6 * (v_des - veh.v[i])
            veh.set_orientation_cmd(yaw_cmd, pitch_cmd)
            veh.step(a_cmd, PHYS_DT)
            energy += sum(x * x for x in a_cmd) * PHYS_DT

            # ---- logging ---------------------------------------------------
            bore, bore_h, bore_v = veh.bore_error(scene.gauge)
            dist = scene.distance_to_gauge(veh.p)
            on_target = math.degrees(bore) < scene.on_target_angle_deg
            for k, key in enumerate(("px", "py", "pz")):
                timeline[key].append(round(veh.p[k], 3))
            for k, key in enumerate(("vx", "vy", "vz")):
                timeline[key].append(round(veh.v[k], 3))
            for k, key in enumerate(("rx", "ry", "rz")):
                timeline[key].append(round(ref[k], 3))
            timeline["t"].append(round(t, 3))
            timeline["yaw"].append(round(veh.att.yaw, 3))
            timeline["pitch"].append(round(veh.att.pitch, 3))
            timeline["roll"].append(round(veh.att.roll, 3))
            timeline["bore"].append(round(math.degrees(bore), 2))
            timeline["bore_h"].append(round(math.degrees(bore_h), 2))
            timeline["bore_v"].append(round(math.degrees(bore_v), 2))
            timeline["dist"].append(round(dist, 3))
            timeline["speed"].append(round(math.sqrt(sum(x * x for x in veh.v)), 3))
            timeline["track_err"].append(round(math.sqrt(
                sum((ref[i] - veh.p[i]) ** 2 for i in range(3))), 3))
            timeline["energy"].append(round(energy, 2))
            timeline["wind"].append(round(math.sqrt(sum(x * x for x in fault.gust_accel)), 2))
            timeline["hold"].append(1 if (holding or rtl_mode) else 0)
            timeline["on_target"].append(1 if on_target else 0)

            if any(abs(t - kt) < PHYS_DT / 2 for kt in key_times):
                keyframes.append({
                    "t": round(t, 3),
                    "bore_h": round(math.degrees(bore_h), 1),
                    "bore_v": round(math.degrees(bore_v), 1),
                    "roll": round(math.degrees(veh.att.roll), 1),
                    "dist": round(dist, 2),
                    "clarity": round(self._clarity(bore, dist, fault.visual_loss), 2),
                })

            # fault window events
            for w in self.faults.windows():
                if abs(t - w["t0"]) < PHYS_DT / 2:
                    event(f"{w['kind']}-start", f"fault active: {w['kind']}")
                if abs(t - w["t1"]) < PHYS_DT / 2:
                    event(f"{w['kind']}-end", f"fault cleared: {w['kind']}")

            # success detection: on station, framed, stable
            d_goal = math.dist(veh.p, scene.standoff)
            if success_t is None and t > 4.0 and d_goal < scene.standoff_tol and on_target:
                if on_target_since is None:
                    on_target_since = t
                elif t - on_target_since >= scene.hold_success_s:
                    success_t = t
                    event("success", f"inspection stable at standoff for "
                                     f"{scene.hold_success_s:.0f} s -> task success")
            else:
                on_target_since = None

            # RTL landed?
            if rtl_mode and veh.p[2] <= 0.14 and math.dist(veh.p[:2], scene.home[:2]) < 0.4:
                event("rtl-landed", "RTL complete: landed at home")
                end_t = t
                break

            t += PHYS_DT

        record = {
            "meta": {
                "flightvla_version": __version__,
                "agent": {"name": self.agent.name, "description": self.agent.description},
                "vehicle": self.vehicle.describe(),
                "faults": self.faults.to_dict(),
                "seed": self.seed,
                "duration": round(end_t, 3),
                "backend": "sim",
            },
            "timeline": timeline,
            "chunks": chunks,
            "events": events,
            "keyframes": keyframes,
            "outcome": {
                "success": success_t is not None,
                "success_t": None if success_t is None else round(success_t, 3),
                "rtl": rtl_mode,
                "holds": sum(1 for e in events if e["kind"] == "hold"),
                "timeouts": sum(1 for c in chunks if c.get("timeout")),
            },
        }
        return record

    # ------------------------------------------------------------------ #
    @staticmethod
    def _push_hold(queue: _RefQueue, t_from: float, p: List[float],
                   yaw: float, pitch: float, t_until: float) -> None:
        t = t_from
        while t < t_until:
            queue.push(t, list(p), yaw, pitch)
            t += 0.2

    @staticmethod
    def _clarity(bore: float, dist: float, visual_loss: bool) -> float:
        if visual_loss:
            return 0.0
        c_angle = max(0.0, 1.0 - math.degrees(bore) / 50.0)
        c_dist = max(0.0, min(1.0, 1.5 - dist / 8.0))
        return round(max(0.0, min(1.0, c_angle * (0.4 + 0.6 * c_dist))), 3)

    @staticmethod
    def _yaw_to(p: List[float], target: List[float]) -> float:
        return math.atan2(target[1] - p[1], target[0] - p[0])
