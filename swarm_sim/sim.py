"""Tier A simulation loop: missions -> guidance -> (separation policy) -> fleet.

One vehicle per mission: it waits on its pad until release, flies the route,
lands, and leaves the simulation. Loss-of-separation (LoS) events are pairs of
airborne vehicles closer than `h_sep_m` horizontally AND `v_sep_m` vertically,
merged over contiguous time; each is tagged "terminal" when either vehicle is
within `terminal_radius_m` of a hub (pad operations), else "en_route".

The separation thresholds are study assumptions, not a regulation: there is no
settled UAS-to-UAS well-clear definition for small drones. Keep them in the
config and report them with every result.
"""
from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field

import numpy as np
from scipy.spatial import cKDTree

from .missions import Mission
from .osm import Airspace
from .vehicle import KinematicFleet, VehicleParams


@dataclass(frozen=True)
class SimParams:
    dt: float = 0.1
    log_hz: float = 2.0
    duration_s: float = 1800.0
    max_extra_s: float = 1800.0         # run past duration until missions finish, at most this long
    arrive_tol_m: float = 1.0
    pass_tol_m: float = 5.0             # pass-through waypoints (no hold, above 15 m): horizontal tolerance,
    pass_tol_z_m: float = 1.0           # ... and vertical (intent boxes are only +-2.5 m vertically)
    h_sep_m: float = 30.0
    v_sep_m: float = 10.0
    terminal_radius_m: float = 40.0


class SeparationPolicy:
    """Adjusts desired velocities for separation. `none` = baseline; see swarm_sim.orca.OrcaPolicy.

    ctx (per row of `ids`): leg_a / leg_b (k,3) current leg endpoints, priority (k,), airspace."""
    name = "none"

    def adjust(self, fleet: KinematicFleet, ids: np.ndarray, desired: np.ndarray, ctx: dict | None = None) -> np.ndarray:
        return desired

    def info(self) -> dict:
        return {}


@dataclass
class SimResult:
    missions: list
    times: np.ndarray               # (L,) log times
    tracks: np.ndarray              # (L, N, 3), NaN when not airborne
    status: list                    # per mission dict
    los_events: list
    intrusions: dict
    wall_s: float
    params: dict = field(default_factory=dict)


def _guidance(pos, wp, vp: VehicleParams):
    d = np.asarray(wp) - pos
    dxy = math.hypot(d[0], d[1])
    vxy = min(vp.cruise_xy, math.sqrt(2 * vp.a_xy_max * dxy) * 0.9) if dxy > 1e-6 else 0.0
    vz_lim = vp.v_up_max if d[2] > 0 else vp.v_down_max
    a_z = vp.a_up_max if d[2] > 0 else vp.a_down_max
    vz = math.copysign(min(vz_lim, math.sqrt(2 * a_z * abs(d[2])) * 0.9), d[2]) if abs(d[2]) > 1e-6 else 0.0
    # climb/descend first when far from the target altitude near the ground (pads, drops)
    if abs(d[2]) > 3.0 and (pos[2] < 15.0 or wp[2] < 15.0) and dxy < 5.0:
        vxy *= 0.3
    v = np.zeros(3)
    if dxy > 1e-6:
        v[:2] = d[:2] / dxy * vxy
    v[2] = vz
    return v


def follow(m: Mission, wi: int, hold_until: float, p, t: float, sp: SimParams) -> tuple[int, float, bool]:
    """Waypoint logic shared by Tier A and the PX4 bridge: (new index, new hold end, advanced?).
    Holds start on arrival; pass-through waypoints above 15 m use the looser pass tolerance."""
    w = m.waypoints[wi]
    if hold_until >= 0:                                  # holding at w (drop, on-scene, inspection)
        if t >= hold_until:
            return wi + 1, -1.0, True
        return wi, hold_until, False
    if (math.hypot(p[0] - w.x, p[1] - w.y) <= sp.pass_tol_m and abs(p[2] - w.z) <= sp.pass_tol_z_m
            if (w.hold_s <= 0 and w.z >= 15.0 and wi + 1 < len(m.waypoints))
            else math.dist(p, (w.x, w.y, w.z)) <= sp.arrive_tol_m):
        if w.hold_s > 0:
            return wi, t + w.hold_s, False
        return wi + 1, -1.0, True
    return wi, hold_until, False


def leg_of(m: Mission, wi: int):
    """(start, end) of the active leg; the first leg starts at meta['start']."""
    w = m.waypoints[wi]
    prev = m.waypoints[wi - 1] if wi > 0 else None
    a = (prev.x, prev.y, prev.z) if prev is not None else tuple(m.meta["start"])
    return a, (w.x, w.y, w.z)


class Simulation:
    """Steppable Tier A simulation. `run()` drives it to completion; the digital twin drives it tick by tick
    as the plant (`step`, `dispatch`, `set_route`, `cancel`) and forks rollouts from measured state
    (`from_state`).

    Missions are indexed by position: vehicle i flies missions[i]. A mission whose `release_t` is +inf
    waits until `dispatch` gives it a release time; `cancel` removes it (rejected requests)."""

    def __init__(self, air: Airspace, missions: list[Mission], vp: VehicleParams = VehicleParams(),
                 sp: SimParams = SimParams(), policy: SeparationPolicy | None = None,
                 monitors: bool = True, log: bool = True):
        self.air, self.vp, self.sp = air, vp, sp
        self.policy = policy or SeparationPolicy()
        self.missions = list(missions)
        n = self.n = len(missions)
        starts = np.array([m.meta["start"] for m in missions], float).reshape(n, 3)
        self.fleet = KinematicFleet(n, vp, starts)
        self.wp_idx = np.zeros(n, int)
        self.hold_until = np.full(n, -1.0)
        self.state = np.array(["waiting"] * n, dtype=object)
        self.t_takeoff = np.full(n, np.nan)
        self.t_landed = np.full(n, np.nan)
        self.wp_times = [[] for _ in range(n)]          # time each waypoint was reached (holds: when the hold ended)
        self.prio = np.array([m.priority for m in missions], int)
        self.hubs_xy = np.array([[h.x, h.y] for h in air.hubs]) if air.hubs else np.zeros((0, 2))
        self.log_every = max(1, round(1.0 / (sp.log_hz * sp.dt)))
        self.monitors, self.log = monitors, log
        self.times, self.tracks = [], []
        self.open_pairs, self.los = {}, []
        self.bld_hits, self.nofly_hits = set(), set()
        self.step_no = 0
        self.wall0 = time.time()

    # ------------------------------------------------------------------ twin hooks
    def dispatch(self, i: int, mission: Mission):
        """Give waiting vehicle i its (scheduled) mission; it takes off at mission.release_t."""
        assert self.state[i] == "waiting", f"vehicle {i} is {self.state[i]}"
        self.missions[i] = mission
        self.prio[i] = mission.priority
        self.fleet.pos[i] = mission.meta["start"]

    def cancel(self, i: int):
        if self.state[i] == "waiting":
            self.state[i] = "cancelled"

    def set_route(self, i: int, mission: Mission):
        """Replace a flying vehicle's remaining route (replan). `mission.waypoints` start from its current
        position; `mission.meta['start']` is that position (the first leg's origin)."""
        self.missions[i] = mission
        self.wp_idx[i] = 0
        self.hold_until[i] = -1.0

    def progress(self, i: int) -> dict:
        return {"wp_idx": int(self.wp_idx[i]), "hold_until": float(self.hold_until[i]), "state": str(self.state[i])}

    @classmethod
    def from_state(cls, air, missions, vp, sp, policy, t, pos, vel, state, wp_idx, hold_until):
        """Fork a simulation from a (measured) state: used by the twin for prediction rollouts."""
        sim = cls(air, missions, vp, sp, policy, monitors=False, log=False)
        sim.fleet.t = t
        sim.fleet.pos[:] = pos
        sim.fleet.vel[:] = vel
        sim.fleet.cmd[:] = vel
        sim.state[:] = state
        sim.fleet.on[:] = state == "flying"
        sim.wp_idx[:] = wp_idx
        sim.hold_until[:] = hold_until
        return sim

    # ------------------------------------------------------------------ one tick
    def step(self) -> bool:
        """Advance one dt. False when nothing is left to fly after the demand window (the run is over)."""
        sp, vp, fleet, missions, state = self.sp, self.vp, self.fleet, self.missions, self.state
        wp_idx, hold_until = self.wp_idx, self.hold_until
        t = fleet.t
        # releases
        for i in np.flatnonzero(state == "waiting"):
            if missions[i].release_t <= t:
                state[i] = "flying"
                fleet.on[i] = True
                self.t_takeoff[i] = t
        fly = np.flatnonzero(state == "flying")
        if len(fly) == 0 and t >= sp.duration_s and not (state == "waiting").any():
            return False
        desired = np.zeros((len(fly), 3))
        leg_a = np.zeros((len(fly), 3))
        leg_b = np.zeros((len(fly), 3))
        for k, i in enumerate(fly):
            m = missions[i]
            p = fleet.pos[i]
            wp_idx[i], hold_until[i], advance = follow(m, int(wp_idx[i]), float(hold_until[i]), p, t, sp)
            if advance:
                self.wp_times[i].append(round(t, 2))
                if wp_idx[i] >= len(m.waypoints):
                    state[i] = "landed"
                    continue
            w = m.waypoints[wp_idx[i]]
            desired[k] = _guidance(p, (w.x, w.y, w.z), vp)   # station-keeps while holding
            leg_a[k], leg_b[k] = leg_of(m, int(wp_idx[i]))
        done = fly[state[fly] == "landed"]
        for i in done:
            fleet.on[i] = False
            fleet.vel[i] = 0
            fleet.cmd[i] = 0
            self.t_landed[i] = t
        keep = state[fly] == "flying"
        fly, desired = fly[keep], desired[keep]
        if len(fly):
            ctx = {"leg_a": leg_a[keep], "leg_b": leg_b[keep], "priority": self.prio[fly], "airspace": self.air,
                   "hubs_xy": self.hubs_xy}
            fleet.command_velocity(fly, self.policy.adjust(fleet, fly, desired, ctx))
        fleet.step(sp.dt)
        self._monitor()
        if self.log and self.step_no % self.log_every == 0:
            self.times.append(fleet.t)
            snap = np.full((self.n, 3), np.nan)
            snap[fleet.on] = fleet.pos[fleet.on]
            self.tracks.append(snap)
        self.step_no += 1
        return True

    def _monitor(self):
        """Building / no-fly intrusions and loss-of-separation events (merged over contiguous time)."""
        fleet, sp, air, hubs_xy = self.fleet, self.sp, self.air, self.hubs_xy
        air_ids = np.flatnonzero(fleet.on & (fleet.pos[:, 2] > 0.5))
        if self.monitors:
            for i in air_ids:
                x, y, z = fleet.pos[i]
                if i not in self.bld_hits and air.inside_building(x, y, z):
                    self.bld_hits.add(int(i))
                if i not in self.nofly_hits and air.in_no_fly(x, y, z):
                    self.nofly_hits.add(int(i))
        current = set()
        open_pairs = self.open_pairs
        if len(air_ids) > 1:
            tree = cKDTree(fleet.pos[air_ids, :2])
            for a, b in tree.query_pairs(sp.h_sep_m):
                i, j = int(air_ids[a]), int(air_ids[b])
                if abs(fleet.pos[i, 2] - fleet.pos[j, 2]) >= sp.v_sep_m:
                    continue
                key = (min(i, j), max(i, j))
                current.add(key)
                d3 = float(np.linalg.norm(fleet.pos[i] - fleet.pos[j]))
                ev = open_pairs.get(key)
                mid = (fleet.pos[i] + fleet.pos[j]) / 2
                term = bool(len(hubs_xy)) and bool(
                    np.min(np.hypot(*(hubs_xy - fleet.pos[i, :2]).T)) < sp.terminal_radius_m
                    or np.min(np.hypot(*(hubs_xy - fleet.pos[j, :2]).T)) < sp.terminal_radius_m)
                if ev is None:
                    open_pairs[key] = {"a": key[0], "b": key[1], "t_start": round(fleet.t, 2), "t_end": round(fleet.t, 2),
                                       "min_dist_m": d3, "x": float(mid[0]), "y": float(mid[1]), "z": float(mid[2]),
                                       "terminal": term}
                else:
                    ev["t_end"] = round(fleet.t, 2)
                    ev["terminal"] = ev["terminal"] or term
                    if d3 < ev["min_dist_m"]:
                        ev.update(min_dist_m=d3, x=float(mid[0]), y=float(mid[1]), z=float(mid[2]))
        for key in list(open_pairs):
            if key not in current:
                self.los.append(open_pairs.pop(key))

    def result(self) -> SimResult:
        los = self.los + list(self.open_pairs.values())
        status = [{"mission": m.id, "kind": m.kind, "release_t": m.release_t, "takeoff_t": _f(self.t_takeoff[i]),
                   "landed_t": _f(self.t_landed[i]), "state": str(self.state[i]), "wp_times": self.wp_times[i]}
                  for i, m in enumerate(self.missions)]
        return SimResult(self.missions, np.array(self.times),
                         np.array(self.tracks) if self.tracks else np.zeros((0, self.n, 3)), status,
                         sorted(los, key=lambda e: e["t_start"]),
                         {"building": sorted(self.bld_hits), "no_fly": sorted(self.nofly_hits)},
                         time.time() - self.wall0,
                         {"vehicle": asdict(self.vp), "sim": asdict(self.sp), "policy": self.policy.name,
                          "policy_info": self.policy.info()})


def run(air: Airspace, missions: list[Mission], vp: VehicleParams = VehicleParams(),
        sp: SimParams = SimParams(), policy: SeparationPolicy | None = None) -> SimResult:
    sim = Simulation(air, missions, vp, sp, policy)
    t_end = sp.duration_s + sp.max_extra_s
    while sim.fleet.t < t_end:
        if not sim.step():
            break
    return sim.result()


def _f(v):
    return None if np.isnan(v) else round(float(v), 2)
