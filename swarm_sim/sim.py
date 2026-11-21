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


def run(air: Airspace, missions: list[Mission], vp: VehicleParams = VehicleParams(),
        sp: SimParams = SimParams(), policy: SeparationPolicy | None = None) -> SimResult:
    policy = policy or SeparationPolicy()
    n = len(missions)
    starts = np.array([m.meta["start"] for m in missions], float).reshape(n, 3)
    fleet = KinematicFleet(n, vp, starts)
    wp_idx = np.zeros(n, int)
    hold_until = np.full(n, -1.0)
    state = np.array(["waiting"] * n, dtype=object)
    t_takeoff = np.full(n, np.nan)
    t_landed = np.full(n, np.nan)
    wp_times = [[] for _ in range(n)]           # time each waypoint was reached (holds: when the hold ended)
    prio = np.array([m.priority for m in missions], int)
    hubs_xy = np.array([[h.x, h.y] for h in air.hubs]) if air.hubs else np.zeros((0, 2))
    log_every = max(1, round(1.0 / (sp.log_hz * sp.dt)))
    times, tracks = [], []
    open_pairs, los = {}, []
    bld_hits, nofly_hits = set(), set()
    t_end = sp.duration_s + sp.max_extra_s
    wall0 = time.time()
    step = 0
    while fleet.t < t_end:
        t = fleet.t
        # releases
        for i in np.flatnonzero(state == "waiting"):
            if missions[i].release_t <= t:
                state[i] = "flying"
                fleet.on[i] = True
                t_takeoff[i] = t
        fly = np.flatnonzero(state == "flying")
        if len(fly) == 0 and t >= sp.duration_s and not (state == "waiting").any():
            break
        desired = np.zeros((len(fly), 3))
        leg_a = np.zeros((len(fly), 3))
        leg_b = np.zeros((len(fly), 3))
        for k, i in enumerate(fly):
            m = missions[i]
            w = m.waypoints[wp_idx[i]]
            p = fleet.pos[i]
            advance = False
            if hold_until[i] >= 0:                       # holding at w (drop, on-scene, inspection)
                if t >= hold_until[i]:
                    hold_until[i] = -1.0
                    advance = True
            elif (math.hypot(p[0] - w.x, p[1] - w.y) <= sp.pass_tol_m and abs(p[2] - w.z) <= sp.pass_tol_z_m
                  if (w.hold_s <= 0 and w.z >= 15.0 and wp_idx[i] + 1 < len(m.waypoints))
                  else math.dist(p, (w.x, w.y, w.z)) <= sp.arrive_tol_m):
                if w.hold_s > 0:
                    hold_until[i] = t + w.hold_s
                else:
                    advance = True
            if advance:
                wp_times[i].append(round(t, 2))
                wp_idx[i] += 1
                if wp_idx[i] >= len(m.waypoints):
                    state[i] = "landed"
                    continue
                w = m.waypoints[wp_idx[i]]
            desired[k] = _guidance(p, (w.x, w.y, w.z), vp)   # station-keeps while holding
            prev = m.waypoints[wp_idx[i] - 1] if wp_idx[i] > 0 else None
            leg_a[k] = (prev.x, prev.y, prev.z) if prev is not None else m.meta["start"]
            leg_b[k] = (w.x, w.y, w.z)
        done = fly[state[fly] == "landed"]
        for i in done:
            fleet.on[i] = False
            fleet.vel[i] = 0
            fleet.cmd[i] = 0
            t_landed[i] = t
        keep = state[fly] == "flying"
        fly, desired = fly[keep], desired[keep]
        if len(fly):
            ctx = {"leg_a": leg_a[keep], "leg_b": leg_b[keep], "priority": prio[fly], "airspace": air,
                   "hubs_xy": hubs_xy}
            fleet.command_velocity(fly, policy.adjust(fleet, fly, desired, ctx))
        fleet.step(sp.dt)
        # safety monitors
        air_ids = np.flatnonzero(fleet.on & (fleet.pos[:, 2] > 0.5))
        for i in air_ids:
            x, y, z = fleet.pos[i]
            if i not in bld_hits and air.inside_building(x, y, z):
                bld_hits.add(int(i))
            if i not in nofly_hits and air.in_no_fly(x, y, z):
                nofly_hits.add(int(i))
        current = set()
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
                los.append(open_pairs.pop(key))
        if step % log_every == 0:
            times.append(fleet.t)
            snap = np.full((n, 3), np.nan)
            snap[fleet.on] = fleet.pos[fleet.on]
            tracks.append(snap)
        step += 1
    los.extend(open_pairs.values())
    status = [{"mission": m.id, "kind": m.kind, "release_t": m.release_t, "takeoff_t": _f(t_takeoff[i]),
               "landed_t": _f(t_landed[i]), "state": str(state[i]), "wp_times": wp_times[i]}
              for i, m in enumerate(missions)]
    return SimResult(missions, np.array(times), np.array(tracks) if tracks else np.zeros((0, n, 3)), status,
                     sorted(los, key=lambda e: e["t_start"]),
                     {"building": sorted(bld_hits), "no_fly": sorted(nofly_hits)}, time.time() - wall0,
                     {"vehicle": asdict(vp), "sim": asdict(sp), "policy": policy.name,
                      "policy_info": policy.info()})


def _f(v):
    return None if np.isnan(v) else round(float(v), 2)
