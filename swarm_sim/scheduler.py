"""Strategic deconfliction: first-come-first-served (FCFS) scheduling of operational intents.

Each flight request is processed in request order. The scheduler searches for the earliest
departure (ground delay in steps of `delay_step_s`, up to `max_delay_s`) and, at each delay, for
the lowest cruise layers (raising the outbound and/or return layer by whole same-direction steps,
2 x 15 m = 30 m) whose en-route intent boxes are cylinder-separated (Theorem 2: horizontal >= 30 m
or vertical >= 10 m, or disjoint in time) from every box already accepted. The first such plan is
accepted and its boxes join the accepted set.

* Emergency flights are never ground-delayed: they try layer changes only, and if none certifies
  they fly anyway ("accepted_uncertified"), with their boxes added so later requests avoid them.
* A delivery or inspection with no certified plan within `max_delay_s` is rejected (not flown).
* Terminal boxes (take-off / landing columns at the pad) are excluded: hub sequencing is plan
  step 7.
* `terminal="sequence"`: departure climbs are certified here; arrivals by the hub sequencer only.
  `terminal="reserve"` (milestone 5.5): the planned arrival (approach fix -> pad) is certified at
  admission too, as a landing *reservation* (a flight is not launched without one); the hub sequencer
  replaces it tactically. Reservation rows are marked in the registry (`arr`) so an emergency may
  displace them (the owner then falls back to tactical sequencing).

The search uses a vectorised Python reference (`separated_hv_matrix`) or, if given, the
Lean-verified C checker. Either way `scripts/audit_run.py` re-checks every accepted pair with the
verified checker afterwards, so the search code is not part of the trusted base.
"""
from __future__ import annotations

import copy
import time
from dataclasses import asdict, dataclass

import numpy as np

from .intents import IntentParams, mission_intents, separated_hv_matrix
from .missions import Mission, RoutingParams, Waypoint, route_clear
from .osm import Airspace
from .vehicle import VehicleParams


@dataclass(frozen=True)
class SchedulerParams:
    h_sep_m: float = 30.0
    v_sep_m: float = 10.0
    delay_step_s: float = 10.0
    max_delay_s: float = 900.0
    max_layer_bump: int = 2               # extra same-direction layers per transit leg
    emergency_max_delay_s: float = 0.0
    priority_max_delay_s: float = 0.0     # reserve: an emergency that moves others may also wait this long on the ground
    reserve_fallback: bool = False        # reserve: no plan with a landing reservation within max_delay_s -> admit
                                          # without one (tactical sequencing only) instead of rejecting
    terminal: str = "exclude"             # exclude: terminal boxes are not certified (milestone 4);
                                          # sequence: departure climbs are certified here (with pad choice),
                                          # arrivals by the hub sequencer (swarm_sim/hubs.py)
                                          # reserve: as sequence, and the planned arrival is certified at
                                          # admission as a landing reservation (milestone 5.5)


@dataclass
class Decision:
    mission: int
    kind: str
    requested_t: float
    scheduled_t: float
    ground_delay_s: float
    bump_out: int
    bump_back: int
    status: str                           # accepted | accepted_uncertified | rejected
    candidates: int


def _bumps(k: int):
    """(out, back) layer bumps ordered by total extra climb, then outbound first."""
    return sorted(((a, b) for a in range(k + 1) for b in range(k + 1)), key=lambda ab: (ab[0] + ab[1], ab[1]))


def variant(m: Mission, air: Airspace, rp: RoutingParams, delay: float, bump_out: int, bump_back: int):
    """Copy of `m` delayed by `delay` s with transit layers raised; None if above the ceiling.
    Outbound: the first climb and cruise waypoints. Return: from the last climb to the waypoint before landing
    (cruise, or fix + approach)."""
    w = m.waypoints
    back = max((i for i, x in enumerate(w) if x.tag == "climb"), default=-1)
    if len(w) < 5 or w[0].tag != "climb" or back < 2 or w[-1].tag != "land":
        return None if (bump_out or bump_back) else _shift(m, delay)
    dz_o, dz_b = 2 * rp.layer_step_m * bump_out, 2 * rp.layer_step_m * bump_back
    if w[0].z + dz_o > air.ceiling_m or w[back].z + dz_b > air.ceiling_m:
        return None
    v = _shift(m, delay)
    for i in (0, 1):
        v.waypoints[i].z += dz_o
    for i in range(back, len(w) - 1):
        v.waypoints[i].z += dz_b
    return v


def with_pad(m: Mission, air: Airspace, rp: RoutingParams, pad: int):
    """Copy of `m` departing from (and, until the hub sequencer says otherwise, returning to) pad `pad` of its
    hub; None if the changed pad legs do not clear buildings."""
    hub = next(h for h in air.hubs if h.name == m.hub)
    px, py = hub.pad_positions()[pad]
    v = copy.deepcopy(m)
    v.pad = pad
    v.meta = dict(v.meta) | {"start": [px, py, hub.elevation]}
    for i, w in enumerate(v.waypoints):
        if i == 0 or w.tag in ("approach", "land") or (w.tag == "cruise" and i == len(v.waypoints) - 2):
            w.x, w.y = px, py
    return v if route_clear(air, tuple(v.meta["start"]), v.waypoints, rp) else None


def _shift(m: Mission, delay: float) -> Mission:
    v = copy.deepcopy(m)
    v.release_t = round(m.release_t + delay, 2)
    return v


class FcfsScheduler:
    """Strategic scheduler over a live registry of accepted en-route intent boxes.

    `schedule(missions)` is the batch form (milestone 4): `admit` in request order. The digital twin
    uses the incremental form: `admit` each request when it arrives, `withdraw` a flight's boxes when it
    is replanned or lands, `replan` a non-conforming flight from its measured position, and `prune`
    boxes that have expired."""
    name = "fcfs"

    def __init__(self, air: Airspace, vp: VehicleParams = VehicleParams(), ip: IntentParams = IntentParams(),
                 rp: RoutingParams = RoutingParams(), params: SchedulerParams = SchedulerParams(), checker=None):
        self.air, self.vp, self.ip, self.rp, self.p = air, vp, ip, rp, params
        self.checker = checker                 # runtime.utm_checker.VerifiedChecker or None
        self.h_mm, self.v_mm = int(round(params.h_sep_m * 1000)), int(round(params.v_sep_m * 1000))
        self.acc = np.zeros((0, 8), np.int64)  # accepted en-route boxes (mm, ms)
        self.owner = np.zeros(0, np.int64)     # flight key per accepted box
        self.arr = np.zeros(0, bool)           # row is a landing reservation (terminal="reserve")

    @property
    def hub_terminal(self) -> bool:
        return self.p.terminal in ("sequence", "reserve")

    def reserved(self, m: Mission) -> bool:
        """The flight's planned arrival is a certified reservation (not yet replaced by a sequenced plan)."""
        return (self.p.terminal == "reserve" and not m.meta.get("sequenced") and not m.meta.get("awaiting")
                and not m.meta.get("unreserved"))

    def certified(self, boxes: list, m: Mission) -> list[bool]:
        """Which of a flight's boxes the scheduler certifies: en route always; with terminal sequencing also the
        departure climb, and the arrival once the hub sequencer has sequenced it (meta['sequenced']), or, with
        reservations, from admission until it is displaced or replaced by a holding plan."""
        if not self.hub_terminal:
            return [b.phase == "en_route" for b in boxes]
        ok = bool(m.meta.get("sequenced")) or self.reserved(m)
        return [b.flow != "arr" or ok for b in boxes]

    def _boxes_arr(self, m: Mission, vp: VehicleParams | None = None):
        """Certified boxes and which of them are a landing reservation."""
        bl = mission_intents(m, vp or self.vp, self.ip)
        res = self.reserved(m)
        sel = [(b.as_int(), res and b.flow == "arr") for b, c in zip(bl, self.certified(bl, m)) if c]
        return (np.array([x for x, _ in sel], np.int64).reshape(-1, 8), np.array([a for _, a in sel], bool))

    def _boxes(self, m: Mission, vp: VehicleParams | None = None) -> np.ndarray:
        bl = mission_intents(m, vp or self.vp, self.ip)
        bx = [b.as_int() for b, c in zip(bl, self.certified(bl, m)) if c]
        return np.array(bx, np.int64).reshape(-1, 8)

    def _conflicts(self, cand: np.ndarray, acc: np.ndarray) -> int:
        if len(cand) == 0 or len(acc) == 0:
            return 0
        lo, hi = cand[:, 6].min(), cand[:, 7].max()
        acc = acc[(acc[:, 7] >= lo) & (acc[:, 6] <= hi)]
        if len(acc) == 0:
            return 0
        if self.checker is not None:
            return self.checker.count_against_hv(cand, acc, self.h_mm, self.v_mm)
        return int((~separated_hv_matrix(cand, acc, self.h_mm, self.v_mm)).sum())

    # ------------------------------------------------------------------ registry
    def register(self, key: int, boxes: np.ndarray, arr: np.ndarray | None = None):
        if len(boxes):
            self.acc = np.vstack([self.acc, boxes])
            self.owner = np.concatenate([self.owner, np.full(len(boxes), key, np.int64)])
            self.arr = np.concatenate([self.arr, np.zeros(len(boxes), bool) if arr is None else np.asarray(arr, bool)])

    def _keep(self, keep: np.ndarray):
        self.acc, self.owner, self.arr = self.acc[keep], self.owner[keep], self.arr[keep]

    def withdraw(self, key: int):
        self._keep(self.owner != key)

    def unreserve(self, key: int):
        """Drop the landing reservation of `key` (its other boxes stay)."""
        self._keep(~((self.owner == key) & self.arr))

    def rows(self, key: int):
        sel = self.owner == key
        return self.acc[sel].copy(), self.arr[sel].copy()

    def displaceable(self, ground=frozenset(), soft=frozenset()) -> np.ndarray:
        """Rows that a priority flight may displace: all boxes of `ground` (departures not yet airborne, which
        are then admitted again) and the reservation rows of `soft` (which then sequence tactically)."""
        return np.isin(self.owner, list(ground)) | (np.isin(self.owner, list(soft)) & self.arr)

    def prune(self, now_s: float):
        self._keep(self.acc[:, 7] >= int(now_s * 1000))

    def conflicts_of(self, key: int, boxes: np.ndarray) -> int:
        """Boxes of `key` that conflict with other flights' accepted boxes (online re-check)."""
        return self._conflicts(boxes, self.acc[self.owner != key])

    # ------------------------------------------------------------------ admission
    def admit(self, m: Mission, key: int | None = None, now: float | None = None, ignore: np.ndarray | None = None,
              max_delay_s: float | None = None):
        """Search the earliest certified plan for request `m` against the registry; register it.
        Returns (variant or None, Decision, boxes). `now` (twin) delays the earliest departure to now.
        `ignore`: registry rows left out of the search (emergency priority; the caller moves their owners)."""
        acc = self.acc if ignore is None else self.acc[~ignore]
        key = m.id if key is None else key
        emergency = m.kind == "emergency"
        max_d = self.p.emergency_max_delay_s if emergency else self.p.max_delay_s
        if max_delay_s is not None:
            max_d = max_delay_s
        n_steps = int(max_d // self.p.delay_step_s)
        k0 = 0 if now is None or now <= m.release_t else int(np.ceil((now - m.release_t) / self.p.delay_step_s - 1e-9))
        chosen, tried, first_ok = None, 0, None
        bases = [m]
        if self.hub_terminal:                       # departure pad choice: own pad first, then the others
            hub = next((h for h in self.air.hubs if h.name == m.hub), None)
            if hub is not None:
                bases += [b for b in (with_pad(m, self.air, self.rp, p) for p in range(hub.pads) if p != m.pad)
                          if b is not None]
        for k in range(k0, max(n_steps, k0) + 1):
            d = k * self.p.delay_step_s
            for (bo, bb), base in ((bb_, b_) for bb_ in _bumps(self.p.max_layer_bump) for b_ in bases):
                v = variant(base, self.air, self.rp, d, bo, bb)
                if v is None:
                    continue
                tried += 1
                if first_ok is None:
                    first_ok = (v, d, bo, bb)
                cb = self._boxes(v)
                if self._conflicts(cb, acc) == 0:
                    chosen = (v, d, bo, bb, cb, "accepted")
                    break
            if chosen:
                break
        if chosen is None and emergency and first_ok is not None:
            v, d, bo, bb = first_ok
            chosen = (v, d, bo, bb, self._boxes(v), "accepted_uncertified")
        if (chosen is None and self.p.terminal == "reserve" and self.p.reserve_fallback and not emergency
                and not m.meta.get("unreserved")):
            m2 = copy.deepcopy(m)
            m2.meta = dict(m2.meta) | {"unreserved": True}
            return self.admit(m2, key, now, ignore, max_delay_s)
        if chosen is None:
            return None, Decision(m.id, m.kind, m.release_t, None, None, 0, 0, "rejected", tried), None
        v, d, bo, bb, cb, status = chosen
        v.meta = dict(v.meta) | {"requested_t": m.release_t, "ground_delay_s": d, "layer_bump": [bo, bb],
                                 "schedule_status": status, "original_id": m.id}
        if self.p.terminal == "reserve":
            cb, arr = self._boxes_arr(v)
            self.register(key, cb, arr if status == "accepted" else None)
        else:
            self.register(key, cb)
        return v, Decision(m.id, m.kind, m.release_t, v.release_t, d, bo, bb, status, tried), cb

    def replan(self, key: int, m: Mission, now: float, max_hover_s: float = 120.0, vp: VehicleParams | None = None):
        """Re-certify an airborne flight from its measured position. `m` is the remaining route with
        meta['start'] = current position; candidates hover there for 0, 10, ... `max_hover_s` s first.
        The flight's old boxes are withdrawn. Returns (mission, status, hover_s, boxes); status is
        'replanned' or, if nothing certifies, 'contingent' (flown anyway, boxes registered). `vp`: this
        vehicle's own (measured) performance, if it differs from the fleet model."""
        self.withdraw(key)
        first = None
        for k in range(int(max_hover_s // self.p.delay_step_s) + 1):
            d = k * self.p.delay_step_s
            v = copy.deepcopy(m)
            v.release_t = now
            x, y, z = v.meta["start"]
            if d > 0:
                v.waypoints = [Waypoint(x, y, z, d, "hover")] + v.waypoints
            cb, arr = self._boxes_arr(v, vp)
            if first is None:
                first = (v, d, cb)
            if self._conflicts(cb, self.acc) == 0:
                self.register(key, cb, arr)
                return v, "replanned", d, cb
        v, d, cb = first
        self.register(key, cb)
        return v, "contingent", 0.0, cb

    # ------------------------------------------------------------------ batch (milestone 4)
    def schedule(self, missions: list[Mission]) -> tuple[list[Mission], dict]:
        t_wall = time.time()
        flown, decisions = [], []
        for m in sorted(missions, key=lambda x: (x.release_t, x.id)):
            v, dec, _ = self.admit(m)
            decisions.append(dec)
            if v is not None:
                flown.append(v)
        flown.sort(key=lambda x: x.release_t)        # renumber: ids must equal list positions in the sim
        for i, v in enumerate(flown):
            v.id = i
        return flown, self.report(decisions, len(missions), time.time() - t_wall)

    def report(self, decisions: list, requests: int, wall_s: float) -> dict:
        stat = {s: sum(x.status == s for x in decisions) for s in ("accepted", "accepted_uncertified", "rejected")}
        delays = [x.ground_delay_s for x in decisions if x.ground_delay_s is not None]
        return {"scheduler": self.name, "params": asdict(self.p),
                "checker": "verified" if self.checker is not None else "python-reference",
                "requests": requests, **stat,
                "ground_delay_s": {"mean": round(float(np.mean(delays)), 2) if delays else None,
                                   "p95": round(float(np.percentile(delays, 95)), 2) if delays else None,
                                   "max": max(delays) if delays else None,
                                   "delayed": int(sum(x > 0 for x in delays))},
                "layer_bumped": sum(1 for x in decisions if x.bump_out or x.bump_back),
                "wall_s": round(wall_s, 2),
                "decisions": [asdict(x) for x in decisions]}
