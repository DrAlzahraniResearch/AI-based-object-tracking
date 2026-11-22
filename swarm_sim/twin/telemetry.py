"""Telemetry and the plant side of the digital twin.

The twin never reads a simulator's internal state. It sees `Telemetry` samples from a `Plant`
(timestamped, noisy, late) and sends commands back (`dispatch`, `set_route`, `cancel`). Three plants:

* `TierAPlant`  - Tier A as the stand-in physical system, with a configurable model mismatch, GNSS-like
                  noise, link latency and fault injection. Ground truth stays inside the plant and is
                  only used to score the twin afterwards.
* `ReplayPlant` - telemetry recorded in a twin log (MCAP), replayed in "shadow mode": commands are
                  logged but cannot change what already happened.
* `Px4Plant`    - PX4 SITL through MAVSDK (swarm_sim/twin/px4.py).
"""
from __future__ import annotations

import copy
import math
import time
from collections import deque
from dataclasses import dataclass, field, replace

import numpy as np

from ..missions import Mission
from ..osm import Airspace
from ..sim import SeparationPolicy, SimParams, Simulation
from ..vehicle import VehicleParams


@dataclass
class Telemetry:
    vid: int
    t: float                      # time the sample was taken (vehicle clock = sim clock)
    pos: np.ndarray               # ENU m (estimate from the autopilot, i.e. noisy)
    vel: np.ndarray               # ENU m/s
    state: str                    # waiting | flying | landed | cancelled
    wp_idx: int                   # active waypoint of the current route (mission progress)
    hold_until: float             # end of the current hold, -1 if not holding
    route_ver: int = 1            # route version the progress fields refer to (1 = as dispatched, +1 per replan)


class Plant:
    """What the twin talks to."""
    name = "plant"

    def now(self) -> float: ...
    def advance(self) -> bool: ...                               # one plant tick; False when finished
    def poll(self) -> list[Telemetry]: ...                       # samples that have arrived since last poll
    def dispatch(self, vid: int, mission: Mission) -> None: ...  # admit a request with its scheduled plan
    def set_route(self, vid: int, mission: Mission) -> None: ... # replace the remaining route
    def cancel(self, vid: int) -> None: ...                      # rejected request
    def truth(self) -> dict: return {}                           # ground truth for scoring (sim plants only)


@dataclass(frozen=True)
class PlantParams:
    rate_hz: float = 5.0              # telemetry rate per vehicle
    latency_s: float = 0.25           # link latency (constant)
    noise_h_m: float = 0.7            # 1-sigma horizontal position noise (autopilot estimate, not raw GNSS)
    noise_v_m: float = 0.5            # 1-sigma vertical
    noise_vel_ms: float = 0.2
    vehicle: dict = field(default_factory=dict)   # VehicleParams overrides: the plant's true dynamics
    faults: tuple = ()                # ({"t": s, "vid": i or "nth_flying": k, "v_xy_max": m/s}, ...)
    seed: int = 0


class TierAPlant(Plant):
    name = "tier_a"

    def __init__(self, air: Airspace, requests: list[Mission], vp_model: VehicleParams, sp: SimParams,
                 policy: SeparationPolicy | None, pp: PlantParams = PlantParams()):
        self.pp = pp
        self.vp_true = replace(vp_model, **pp.vehicle)
        pending = []
        for m in requests:
            c = copy.deepcopy(m)
            c.release_t = math.inf                 # nothing flies until the twin dispatches it
            pending.append(c)
        self.sim = Simulation(air, pending, self.vp_true, sp, policy)
        self.rng = np.random.default_rng(pp.seed)
        self.every = max(1, round(1.0 / (pp.rate_hz * sp.dt)))
        self.queue: deque = deque()               # (deliver_at, Telemetry)
        self.faults = list(pp.faults)
        self.fault_log: list = []
        self.route_ver = np.zeros(self.sim.n, int)
        self.reported_landed = np.zeros(self.sim.n, bool)

    def now(self):
        return self.sim.fleet.t

    def _apply_faults(self):
        t = self.now()
        for f in [f for f in self.faults if f["t"] <= t]:
            fly = np.flatnonzero((self.sim.state == "flying") & (self.sim.fleet.pos[:, 2] > 20))
            vid = f.get("vid")
            if vid is None and len(fly):
                vid = int(fly[min(f.get("nth_flying", 0), len(fly) - 1)])
            if vid is None:
                continue                                  # nobody airborne yet: wait
            self.faults.remove(f)
            self.sim.fleet.v_cap[vid] = f["v_xy_max"]
            self.fault_log.append({"t": round(t, 2), "vid": int(vid), "v_xy_max": f["v_xy_max"]})

    def advance(self) -> bool:
        self._apply_faults()
        alive = self.sim.step()
        if self.sim.step_no % self.every == 0:
            self._sample()
        return alive

    def _sample(self):
        f, s, pp = self.sim.fleet, self.sim, self.pp
        t = f.t
        for i in np.flatnonzero(s.state == "flying"):
            pos = f.pos[i] + self.rng.normal(0, [pp.noise_h_m, pp.noise_h_m, pp.noise_v_m])
            vel = f.vel[i] + self.rng.normal(0, pp.noise_vel_ms, 3)
            self.queue.append((t + pp.latency_s, Telemetry(int(i), t, pos, vel, "flying", int(s.wp_idx[i]),
                                                           float(s.hold_until[i]), int(self.route_ver[i]))))
        for i in np.flatnonzero((s.state == "landed") & ~self.reported_landed):
            self.reported_landed[i] = True                    # report landing once
            if True:
                self.queue.append((t + pp.latency_s, Telemetry(int(i), t, f.pos[i].copy(), np.zeros(3), "landed",
                                                               int(s.wp_idx[i]), -1.0, int(self.route_ver[i]))))

    def poll(self):
        out, t = [], self.now()
        while self.queue and self.queue[0][0] <= t + 1e-9:
            out.append(self.queue.popleft()[1])
        return out

    def dispatch(self, vid, mission):
        self.sim.dispatch(vid, mission)
        self.route_ver[vid] = 1

    def set_route(self, vid, mission):
        self.sim.set_route(vid, mission)
        self.route_ver[vid] += 1

    def cancel(self, vid):
        self.sim.cancel(vid)

    def truth(self):
        return {"pos": self.sim.fleet.pos.copy(), "on": self.sim.fleet.on.copy(), "t": self.now()}


class PacedPlant:
    """Holds a fast plant (Tier A, replay) to `pace` x real time, so a live viewer can follow it.

    A proxy: every attribute but `advance` is the wrapped plant's, so the twin cannot tell the difference."""

    def __init__(self, plant: Plant, pace: float, sleep=time.sleep, clock=time.monotonic):
        self.plant, self.pace, self._sleep, self._clock = plant, float(pace), sleep, clock
        self._w0 = self._t0 = None

    def __getattr__(self, name):
        return getattr(self.plant, name)

    def advance(self) -> bool:
        alive = self.plant.advance()
        t = self.plant.now()
        if self._w0 is None:
            self._w0, self._t0 = self._clock(), t
        ahead = (t - self._t0) / self.pace - (self._clock() - self._w0)
        if ahead > 0:
            self._sleep(ahead)
        return alive


class ReplayPlant(Plant):
    """Replays /twin/telemetry from a twin log in shadow mode (commands are recorded, not applied)."""
    name = "replay"

    def __init__(self, frames: list[tuple[float, list[Telemetry]]], dt: float = 0.1):
        self.frames = deque(sorted(frames, key=lambda f: f[0]))
        self.t = self.frames[0][0] - dt if self.frames else 0.0
        self.dt = dt
        self.ignored: list = []

    def now(self):
        return self.t

    def advance(self):
        self.t += self.dt
        return bool(self.frames)

    def poll(self):
        out = []
        while self.frames and self.frames[0][0] <= self.t + 1e-9:
            out.extend(self.frames.popleft()[1])
        return out

    def dispatch(self, vid, mission):
        self.ignored.append(("dispatch", vid, round(self.t, 2)))

    def set_route(self, vid, mission):
        self.ignored.append(("set_route", vid, round(self.t, 2)))

    def cancel(self, vid):
        self.ignored.append(("cancel", vid, round(self.t, 2)))
