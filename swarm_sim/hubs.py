"""Hub sequencing (plan step 7): tactical landing slots with holding, certified like everything else.

Under the 30 m / 10 m separation standard every pad of a compact hub is within 30 m of every other, so a hub
behaves like a single runway: two operations at the hub are separated only by time or by 10 m of altitude.
The traffic manager therefore controls the terminal area in two places:

* Departures (strategic, `FcfsScheduler` with `terminal="sequence"`): the climb at the pad is certified with
  the rest of the plan when the request is admitted, and the scheduler may pick any pad of the hub.
* Arrivals (tactical, `HubSequencer`): inbound flights end their strategic plan at an approach fix outside
  the terminal area (`RoutingParams.approach_fix_m`, on the inbound bearing). About `lead_s` before the fix,
  the flight asks for a landing slot. The sequencer searches holding time (0, 5, ... `max_hold_s` s), holding
  point (on a ring at the fix radius: the fix and +-60 / +-120 deg; descending there to the approach altitude,
  25 m, or 15 m above it, or staying at the inbound layer) and landing pad, and accepts the first plan whose boxes (current position -> hold -> approach -> land, with
  fresh, tight time windows) are cylinder-separated (Theorem 2) from every box in force, terminal boxes
  included. With nothing certified within `max_hold_s` the flight lands anyway (`contingent`, counted).

The ring radius must keep a holding box (20 m buffer) 30 m clear of every pad column (5 m buffer): at least
(farthest pad from the centre) + 55 m. The compact mini_town hubs need 66 m, the spread layout (40 m pad
spacing) 100 m; milestone 5 uses 100 m everywhere so all layouts fly the same missions. Descending on the
ring, not in the hub's column, keeps a landing's column time to ~17 s (25 m at 1.5 m/s) instead of 30-80 s
from the cruise layers.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from .missions import Mission, RoutingParams, Waypoint, route_clear
from .scheduler import FcfsScheduler
from .vehicle import VehicleParams


@dataclass(frozen=True)
class HubParams:
    lead_s: float = 20.0                 # request a landing slot this long before reaching the fix
    hold_step_s: float = 5.0
    max_hold_s: float = 300.0
    ring_offsets_deg: tuple = (0.0, 60.0, -60.0, 120.0, -120.0)   # 60 deg apart on a ring of radius R = R m apart:
                                         # holding boxes (20 m buffers) at least 30 m clear when R >= 70 m
    approach_alt_m: float = 25.0         # descend at the holding point to this altitude, then cross to the pad
    stack_m: tuple = (0.0, 15.0)         # holding altitudes above the approach altitude
    stack_above_m: float = 30.0          # and one same-direction level above the inbound layer
    retry_s: float = 2.0                 # nothing certifies yet: keep the current plan and retry this often ...
    final_eta_s: float = 3.0             # ... until this close to the fix
    pre_holds_s: tuple = (10.0, 20.0, 40.0)   # then, last resort: hover where it is first
    max_pads: int = 3                    # own pad + the farthest others
    bump_after_s: float = 60.0           # arrival priority: if no slot with at most this much holding, departures
                                         # still on the ground (and not about to lift off) may be moved
    await_retry_s: float = 5.0           # no landing slot at the fix: hold at a certified ring point and retry this
                                         # often, until max_hold_s in total; then land anyway (contingent)


@dataclass
class ArrivalDecision:
    vid: int
    hub: str
    t: float
    status: str                          # sequenced | awaiting (holding at the fix, retrying) | contingent
    hold_s: float
    pre_hold_s: float
    slot: tuple                          # (ring offset deg, altitude offset m)
    pad: int
    own_pad: int
    candidates: int
    bumped: tuple = ()


class HubSequencer:
    def __init__(self, sched: FcfsScheduler, rp: RoutingParams, hp: HubParams = HubParams()):
        self.sched, self.air, self.rp, self.hp = sched, sched.air, rp, hp
        self.hubs = {h.name: h for h in self.air.hubs}
        self.decisions: list[ArrivalDecision] = []

    @staticmethod
    def fix_index(m: Mission) -> int | None:
        return next((i for i, w in enumerate(m.waypoints) if w.tag == "fix"), None)

    def eta(self, m: Mission, wp_idx: int, pos, vel) -> float | None:
        """Seconds to the approach fix for an unsequenced flight on its way there (0 if past it), else None."""
        if m.meta.get("sequenced"):
            return None
        if m.meta.get("awaiting"):
            return 0.0
        f = self.fix_index(m)
        if f is None or wp_idx < f:
            return None
        if wp_idx > f:
            return 0.0
        w = m.waypoints[f]
        return math.hypot(w.x - pos[0], w.y - pos[1]) / max(math.hypot(vel[0], vel[1]), 1.0)

    def due(self, m: Mission, wp_idx: int, pos, vel) -> bool:
        e = self.eta(m, wp_idx, pos, vel)
        return e is not None and e <= self.hp.lead_s

    def _route(self, m: Mission, pos, now: float, slot_xyz, pad: int, hold: float, z_in: float | None = None,
               pre: float = 0.0) -> Mission:
        """[hover `pre` s where it is ->] holding point at the inbound altitude `z_in` -> down to the holding
        altitude (slot z), hold -> across to the pad at that altitude -> land."""
        hub = self.hubs[m.hub]
        px, py = hub.pad_positions()[pad]
        x, y, z = slot_xyz
        wps = [Waypoint(float(pos[0]), float(pos[1]), float(pos[2]), pre, "hover")] if pre > 0 else []
        wps.append(Waypoint(x, y, z if z_in is None else z_in, 0.0 if z_in is not None and z_in != z else hold, "fix"))
        if z_in is not None and z_in != z:
            wps.append(Waypoint(x, y, z, hold, "hold"))
        return replace(m, release_t=now, pad=pad,
                       waypoints=wps + [Waypoint(px, py, z, tag="approach"), Waypoint(px, py, hub.elevation, tag="land")],
                       meta=dict(m.meta) | {"start": [float(c) for c in pos], "pad": [px, py, hub.elevation],
                                            "sequenced": True})

    def _await_route(self, m: Mission, pos, now: float, slot_xyz, hold: float, z_in: float) -> Mission:
        """Holding only: ring point at the inbound altitude -> down to the slot altitude and hold (tagged "fix",
        so the holding is certified and what follows, approach and landing on the planned pad, is not)."""
        hub = self.hubs[m.hub]
        px, py = hub.pad_positions()[m.pad]
        x, y, z = slot_xyz
        wps = ([Waypoint(x, y, z_in, 0.0, "ring")] if z_in != z else []) + [Waypoint(x, y, z, hold, "fix"),
               Waypoint(px, py, z, tag="approach"), Waypoint(px, py, hub.elevation, tag="land")]
        return replace(m, release_t=now, waypoints=wps,
                       meta=dict(m.meta) | {"start": [float(c) for c in pos], "pad": [px, py, hub.elevation],
                                            "sequenced": False, "awaiting": True,
                                            "await_since": m.meta.get("await_since", now)})

    def _geoms(self, m: Mission, pos, now: float):
        hub = self.hubs[m.hub]
        f = self.fix_index(m)
        fx, fy, fz = m.waypoints[f].x, m.waypoints[f].y, m.waypoints[f].z
        th0 = math.atan2(fy - hub.y, fx - hub.x)
        R = math.hypot(fx - hub.x, fy - hub.y)
        pp = hub.pad_positions()
        own = m.pad
        others = sorted((p for p in range(hub.pads) if p != own),
                        key=lambda p: -math.dist(pp[p], pp[own]))[:max(0, self.hp.max_pads - 1)]
        def build(alts):
            out = []
            for zh in alts:
                for off in self.hp.ring_offsets_deg:
                    th = th0 + math.radians(off)
                    slot = (hub.x + R * math.cos(th), hub.y + R * math.sin(th), zh)
                    for p in [own] + others:
                        r0 = self._route(m, pos, now, slot, p, 0.0, fz)
                        if route_clear(self.air, tuple(r0.meta["start"]), r0.waypoints, self.rp):
                            out.append(((off, round(zh - fz, 1)), slot, p))
            return out

        # hold low (approach altitude, or a level above), at the inbound layer, or stacked one same-direction
        # level above it (30 m: the flights behind on the same bearing pass underneath)
        low = sorted({min(fz, self.hp.approach_alt_m + dz) for dz in self.hp.stack_m})
        alts = low + ([fz] if fz not in low else [])
        if fz + self.hp.stack_above_m <= self.air.ceiling_m:
            alts.append(fz + self.hp.stack_above_m)
        return build(alts), fz

    def _holds(self, max_hold: float) -> list[float]:
        """0, 5, ... 60 s, then every 10 s up to max_hold."""
        st = self.hp.hold_step_s
        short = [k * st for k in range(int(min(max_hold, 60.0) // st) + 1)]
        return short + [float(h) for h in range(int(short[-1]) + 10, int(max_hold) + 1, 10)]

    def _search(self, key, m, pos, now, vp, geoms, fz, pre, tried, max_hold=None, bumpable=frozenset()):
        acc, own = self.sched.acc, self.sched.owner
        if bumpable:
            keep = ~np.isin(own, list(bumpable))
            acc_all, own_all = acc, own
            acc = acc[keep]
        alive = list(geoms)
        max_hold = self.hp.max_hold_s if max_hold is None else max_hold
        for k, hold in enumerate(self._holds(max_hold)):
            nxt = []
            for g in alive:
                (off, dz), slot, p = g
                v = self._route(m, pos, now, slot, p, hold, fz, pre)
                cb = self.sched._boxes(v, vp)
                tried[0] += 1
                if self.sched._conflicts(cb, acc) == 0:
                    bumped = ()
                    if bumpable:                 # the ground departures this plan actually displaces
                        bumped = tuple(sorted(int(o) for o in set(own_all.tolist()) & set(bumpable)
                                              if self.sched._conflicts(cb, acc_all[own_all == o]) > 0))
                    self.sched.register(key, cb)
                    waited = now - m.meta.get("await_since", now)
                    v.meta = dict(v.meta) | {"awaiting": False}
                    self.decisions.append(ArrivalDecision(key, m.hub, round(now, 2), "sequenced", hold + waited, pre,
                                                          (off, dz), p, m.pad, tried[0], bumped))
                    return v, "sequenced", hold + pre + waited
                if k == 0 and self._prefix_blocked(m, pos, now, slot, p, vp, acc, fz, pre):
                    continue                     # the way to the hold point conflicts for every hold time
                nxt.append(g)
            alive = nxt
            if not alive:
                return None
        return None

    def sequence(self, key: int, m: Mission, pos, now: float, vp: VehicleParams | None = None, final: bool = True,
                 bumpable: frozenset = frozenset()):
        """Plan and certify the landing of flight `key` (route `m`, measured position `pos`). Returns
        (new route, status, delay s), with the flight's previous boxes replaced by the new ones, or None when
        nothing certifies and `final` is False (the previous plan stays registered; retry later).
        `bumpable`: ground departures that may be displaced (arrival priority) if no slot needs at most
        `bump_after_s` of holding; the caller re-admits `decisions[-1].bumped`."""
        geoms, fz = self._geoms(m, pos, now)
        own = self.sched.owner == key
        saved = self.sched.acc[own].copy()
        self.sched.withdraw(key)
        tried = [0]
        waiting = bool(m.meta.get("awaiting"))
        waited = now - m.meta.get("await_since", now)
        max_hold = max(0.0, self.hp.max_hold_s - waited)
        # early attempts: short holds only. Final (at the fix): long holds, moving ground departures, hovering
        # first. Already holding at the fix: long holds and moving departures.
        passes = [(0.0, min(self.hp.bump_after_s, max_hold), frozenset())]
        if final:
            passes += [(0.0, max_hold, bumpable)]
            if not waiting:
                passes += [(pre, max_hold, bumpable) for pre in self.hp.pre_holds_s]
        for pre, mh, bump in passes:
            res = self._search(key, m, pos, now, vp, geoms, fz, pre, tried, mh, bump)
            if res is not None:
                return res
        if not final:
            self.sched.register(key, saved)
            return None
        f = self.fix_index(m)
        w = m.waypoints[f]
        remaining = self.hp.max_hold_s - waited
        for H in [h for h in (remaining, 120.0, 60.0, 30.0, 15.0) if 10.0 <= h <= remaining]:
            # no landing slot: hold on the ring (outside the terminal area) where a holding box is free for H s,
            # certified like any plan, and retry. The landing part stays uncertified until a slot is found.
            for (off, dz), slot, _p in {g[:2]: g for g in geoms}.values():
                v = self._await_route(m, pos, now, slot, H, fz)
                cb = self.sched._boxes(v, vp)
                tried[0] += 1
                if self.sched._conflicts(cb, self.sched.acc) == 0:
                    self.sched.register(key, cb)
                    self.decisions.append(ArrivalDecision(key, m.hub, round(now, 2), "awaiting", H, 0.0, (off, dz),
                                                          m.pad, m.pad, tried[0]))
                    return v, "awaiting", H
        # holding limit reached: land anyway, from the fix, on the planned pad
        v = self._route(m, pos, now, (w.x, w.y, w.z), m.pad, 0.0)
        v.meta = dict(v.meta) | {"awaiting": False}
        self.sched.register(key, self.sched._boxes(v, vp))
        self.decisions.append(ArrivalDecision(key, m.hub, round(now, 2), "contingent", 0.0, 0.0, (0.0, 0.0), m.pad,
                                              m.pad, tried[0]))
        return v, "contingent", 0.0

    def _prefix_blocked(self, m, pos, now, slot, p, vp, acc, z_in, pre=0.0) -> bool:
        """Boxes of the way to the hold point and the descent there (before any holding) are the same for every
        hold time."""
        v = self._route(m, pos, now, slot, p, 0.0, z_in, pre)
        v.waypoints = v.waypoints[:-2]
        if v.waypoints[-1].tag != "hover":
            v.waypoints[-1].hold_s = 0.0
        cb = self.sched._boxes(v, vp)
        return len(cb) > 0 and self.sched._conflicts(cb, acc) > 0

    def report(self) -> dict:
        d = [x for x in self.decisions if x.status != "awaiting"]      # one final decision per arrival
        holds = [x.hold_s + x.pre_hold_s for x in d if x.status == "sequenced"]
        by_hub = {}
        for x in d:
            by_hub.setdefault(x.hub, 0)
            by_hub[x.hub] += 1
        return {"arrivals": len(d), "sequenced": sum(x.status == "sequenced" for x in d),
                "awaited": len({x.vid for x in self.decisions if x.status == "awaiting"}),
                "contingent": sum(x.status == "contingent" for x in d),
                "hold_s": {"mean": round(float(np.mean(holds)), 1) if holds else None,
                           "p95": round(float(np.percentile(holds, 95)), 1) if holds else None,
                           "max": max(holds) if holds else None,
                           "held": int(sum(h > 0 for h in holds))},
                "off_bearing_slots": sum(1 for x in d if x.status == "sequenced" and x.slot[0] != 0.0),
                "pre_holds": sum(1 for x in d if x.status == "sequenced" and x.pre_hold_s > 0),
                "pad_changes": sum(1 for x in d if x.status == "sequenced" and x.pad != x.own_pad),
                "departures_moved": sum(len(x.bumped) for x in d),
                "by_hub": by_hub, "candidates_mean": round(float(np.mean([x.candidates for x in d])), 1) if d else None}
