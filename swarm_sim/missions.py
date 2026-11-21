"""Civilian missions (delivery, emergency, inspection) and milestone-1 routing.

Routing here is deliberately minimal ("direct layered"): climb at the pad,
fly a straight leg at a cruise layer that clears every building under the leg
by `clearance_m`, descend at the target. Cruise layers follow a semicircular
rule (eastbound/westbound on alternating layers) so opposite directions are
vertically separated by construction. Straight legs that would cross a no-fly
zone are rejected (the generator resamples); routing around no-fly zones is
milestone 2 (scripts/..., docs/architecture.md).
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field

import numpy as np
from shapely.geometry import LineString, Point, Polygon

from .osm import Airspace


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    hold_s: float = 0.0
    tag: str = ""


@dataclass
class Mission:
    id: int
    kind: str                 # delivery | emergency | inspection
    priority: int             # 0 = highest (emergency)
    release_t: float          # s, when the mission may take off
    hub: str
    pad: int
    waypoints: list = field(default_factory=list)
    meta: dict = field(default_factory=dict)

    def to_json(self):
        return asdict(self)


@dataclass(frozen=True)
class RoutingParams:
    layer_base_m: float = 40.0
    layer_step_m: float = 15.0          # > v_sep (10 m) + altitude-keeping tolerance; see decision S11
    clearance_m: float = 15.0          # vertical clearance over buildings under a leg
    corridor_radius_m: float = 15.0    # lateral radius searched for buildings under a leg
    delivery_drop_alt_m: float = 5.0
    delivery_hold_s: float = 20.0
    emergency_alt_m: float = 25.0      # 15 m below the base layer (30 m sat exactly at the 10 m LoS threshold)
    emergency_hold_s: float = 60.0
    inspection_standoff_m: float = 15.0
    inspection_above_m: float = 10.0
    inspection_points: int = 8
    inspection_hold_s: float = 3.0
    leg_check_radius_m: float = 5.0    # every leg must clear buildings within this radius ...
    leg_check_margin_m: float = 2.0    # ... by this much below the leg's lowest altitude
    no_fly_margin_m: float = 20.0      # every leg keeps this far from no-fly zones (= intent lateral buffer,
                                       # > the 15 m ORCA corridor)


def cruise_layer(air: Airspace, a, b, rp: RoutingParams) -> float | None:
    """Lowest semicircular layer for leg a->b that clears buildings; None if above the ceiling."""
    heading = math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) % 360      # 0 = north, clockwise
    parity = 0 if heading < 180 else 1                                      # east-ish odd/even
    need = air.building_top_along(a, b, rp.corridor_radius_m) + rp.clearance_m
    k = parity
    while True:
        z = rp.layer_base_m + k * rp.layer_step_m
        if z > air.ceiling_m:
            return None
        if z >= need:
            return z
        k += 2


def crosses_no_fly(air: Airspace, a, b, margin: float = 0.0) -> bool:
    """True if segment a-b (or point a == b) comes within `margin` m of a no-fly zone."""
    seg = LineString([a, b]) if tuple(a) != tuple(b) else Point(a)
    return any(Polygon(n.polygon).distance(seg) <= margin for n in air.no_fly)


def out_and_back(air: Airspace, start, target_wps: list, rp: RoutingParams):
    """start (x, y, z_pad) -> climb -> cruise -> target waypoints -> cruise -> back to start. None if infeasible."""
    sx, sy, sz = start
    tx, ty = target_wps[0].x, target_wps[0].y
    if crosses_no_fly(air, (sx, sy), (tx, ty), rp.no_fly_margin_m):
        return None
    z_out = cruise_layer(air, (sx, sy), (tx, ty), rp)
    z_back = cruise_layer(air, (tx, ty), (sx, sy), rp)
    if z_out is None or z_back is None:
        return None
    wps = [Waypoint(sx, sy, z_out, tag="climb"), Waypoint(tx, ty, z_out, tag="cruise")]
    for w in target_wps:
        if air.inside_building(w.x, w.y, w.z) or air.in_no_fly(w.x, w.y, w.z):
            return None
    wps += target_wps
    lx, ly = target_wps[-1].x, target_wps[-1].y
    wps += [Waypoint(lx, ly, z_back, tag="climb"), Waypoint(sx, sy, z_back, tag="cruise"),
            Waypoint(sx, sy, sz, tag="land")]
    return wps if route_clear(air, start, wps, rp) else None


def route_clear(air: Airspace, start, wps: list, rp: RoutingParams) -> bool:
    """Every leg (incl. orbits, descents, pad legs) clears nearby buildings below its lowest altitude."""
    prev = start
    for w in wps:
        cur = (w.x, w.y, w.z)
        top = air.building_top_along(prev[:2], cur[:2], rp.leg_check_radius_m)
        if top > 0 and top > min(prev[2], cur[2]) - rp.leg_check_margin_m:
            return False
        if crosses_no_fly(air, prev[:2], cur[:2], rp.no_fly_margin_m):
            return False
        prev = cur
    return True


def _road_point(air: Airspace, rng):
    if air.roads:
        r = air.roads[rng.integers(len(air.roads))]
        k = rng.integers(len(r.line) - 1)
        (x0, y0), (x1, y1) = r.line[k], r.line[k + 1]
        u = rng.uniform()
        return x0 + u * (x1 - x0), y0 + u * (y1 - y0)
    xmin, ymin, xmax, ymax = air.bounds
    return rng.uniform(xmin, xmax), rng.uniform(ymin, ymax)


def _inside_bounds(air, x, y, margin=10.0):
    xmin, ymin, xmax, ymax = air.bounds
    return xmin + margin <= x <= xmax - margin and ymin + margin <= y <= ymax - margin


def generate_missions(air: Airspace, rates_per_h: dict, duration_s: float, seed: int,
                      rp: RoutingParams = RoutingParams(), max_tries: int = 50) -> list[Mission]:
    """Poisson demand per mission kind over [0, duration_s). Deterministic for a seed."""
    if not air.hubs:
        raise ValueError("airspace has no hubs; pass --hubs to osm_to_world.py")
    rng = np.random.default_rng(seed)
    releases = []
    for kind, rate in rates_per_h.items():
        t = 0.0
        while rate > 0:
            t += rng.exponential(3600.0 / rate)
            if t >= duration_s:
                break
            releases.append((t, kind))
    releases.sort()
    missions, pad_next = [], {h.name: 0 for h in air.hubs}
    for t, kind in releases:
        for _ in range(max_tries):
            m = _make(air, kind, t, rng, rp, pad_next)
            if m is not None:
                m.id = len(missions)
                missions.append(m)
                break
    return missions


def _make(air, kind, t, rng, rp, pad_next):
    if kind == "inspection":
        if not air.buildings:
            return None
        b = air.buildings[rng.integers(len(air.buildings))]
        poly = Polygon(b.polygon)
        c = poly.centroid
        r = max(Point(c).distance(Point(p)) for p in b.polygon) + rp.inspection_standoff_m
        ang0 = rng.uniform(0, 2 * math.pi)
        ring = [(c.x + r * math.cos(ang0 + 2 * math.pi * k / rp.inspection_points),
                 c.y + r * math.sin(ang0 + 2 * math.pi * k / rp.inspection_points))
                for k in range(rp.inspection_points + 1)]
        # orbit above the inspected building AND above any neighbour the ring passes near
        ring_top = max(air.building_top_along(a, bb, rp.leg_check_radius_m) for a, bb in zip(ring[:-1], ring[1:]))
        z = max(b.height + rp.inspection_above_m, ring_top + rp.leg_check_margin_m + 1.0)
        if z > air.ceiling_m:
            return None
        target = [Waypoint(x, y, z, rp.inspection_hold_s, "inspect") for x, y in ring]
        if any(not _inside_bounds(air, w.x, w.y) for w in target):
            return None
        meta = {"building": b.osm_id, "building_height_m": b.height}
        prio = 2
    else:
        x, y = _road_point(air, rng)
        if not _inside_bounds(air, x, y):
            return None
        if kind == "delivery":
            target = [Waypoint(x, y, rp.delivery_drop_alt_m, rp.delivery_hold_s, "drop")]
            prio = 1
        else:
            target = [Waypoint(x, y, rp.emergency_alt_m, rp.emergency_hold_s, "on_scene")]
            prio = 0
        meta = {}
    tx, ty = target[0].x, target[0].y
    if kind == "emergency":       # nearest hub
        hub = min(air.hubs, key=lambda h: math.hypot(h.x - tx, h.y - ty))
    else:
        hub = air.hubs[rng.integers(len(air.hubs))]
    pad = pad_next[hub.name] % max(1, hub.pads)
    px, py = hub.pad_positions()[pad]
    wps = out_and_back(air, (px, py, hub.elevation), target, rp)
    if wps is None:
        return None
    pad_next[hub.name] += 1
    return Mission(-1, kind, prio, round(t, 2), hub.name, pad, wps,
                   meta | {"start": [px, py, hub.elevation]})
