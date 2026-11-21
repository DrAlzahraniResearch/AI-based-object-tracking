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
from .missions import Mission, RoutingParams
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
    """Copy of `m` delayed by `delay` s with transit layers raised; None if above the ceiling."""
    w = m.waypoints
    if len(w) < 5 or w[0].tag != "climb" or w[-3].tag != "climb":
        return None if (bump_out or bump_back) else _shift(m, delay)
    dz_o, dz_b = 2 * rp.layer_step_m * bump_out, 2 * rp.layer_step_m * bump_back
    if w[0].z + dz_o > air.ceiling_m or w[-3].z + dz_b > air.ceiling_m:
        return None
    v = _shift(m, delay)
    for i in (0, 1):
        v.waypoints[i].z += dz_o
    for i in (-3, -2):
        v.waypoints[i].z += dz_b
    return v


def _shift(m: Mission, delay: float) -> Mission:
    v = copy.deepcopy(m)
    v.release_t = round(m.release_t + delay, 2)
    return v


class FcfsScheduler:
    name = "fcfs"

    def __init__(self, air: Airspace, vp: VehicleParams = VehicleParams(), ip: IntentParams = IntentParams(),
                 rp: RoutingParams = RoutingParams(), params: SchedulerParams = SchedulerParams(), checker=None):
        self.air, self.vp, self.ip, self.rp, self.p = air, vp, ip, rp, params
        self.checker = checker                 # runtime.utm_checker.VerifiedChecker or None
        self.h_mm, self.v_mm = int(round(params.h_sep_m * 1000)), int(round(params.v_sep_m * 1000))

    def _boxes(self, m: Mission) -> np.ndarray:
        bx = [b.as_int() for b in mission_intents(m, self.vp, self.ip) if b.phase == "en_route"]
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

    def schedule(self, missions: list[Mission]) -> tuple[list[Mission], dict]:
        t_wall = time.time()
        acc = np.zeros((0, 8), np.int64)
        flown, decisions = [], []
        for m in sorted(missions, key=lambda x: (x.release_t, x.id)):
            emergency = m.kind == "emergency"
            max_d = self.p.emergency_max_delay_s if emergency else self.p.max_delay_s
            n_steps = int(max_d // self.p.delay_step_s)
            chosen, tried, first_ok = None, 0, None
            for k in range(n_steps + 1):
                d = k * self.p.delay_step_s
                for bo, bb in _bumps(self.p.max_layer_bump):
                    v = variant(m, self.air, self.rp, d, bo, bb)
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
            if chosen is None:
                decisions.append(Decision(m.id, m.kind, m.release_t, None, None, 0, 0, "rejected", tried))
                continue
            v, d, bo, bb, cb, status = chosen
            v.meta = dict(v.meta) | {"requested_t": m.release_t, "ground_delay_s": d, "layer_bump": [bo, bb],
                                     "schedule_status": status, "original_id": m.id}
            flown.append(v)
            acc = np.vstack([acc, cb]) if len(cb) else acc
            decisions.append(Decision(m.id, m.kind, m.release_t, v.release_t, d, bo, bb, status, tried))
        flown.sort(key=lambda x: x.release_t)        # renumber: ids must equal list positions in the sim
        for i, v in enumerate(flown):
            v.id = i
        stat = {s: sum(x.status == s for x in decisions) for s in ("accepted", "accepted_uncertified", "rejected")}
        delays = [x.ground_delay_s for x in decisions if x.ground_delay_s is not None]
        report = {"scheduler": self.name, "params": asdict(self.p),
                  "checker": "verified" if self.checker is not None else "python-reference",
                  "requests": len(missions), **stat,
                  "ground_delay_s": {"mean": round(float(np.mean(delays)), 2) if delays else None,
                                     "p95": round(float(np.percentile(delays, 95)), 2) if delays else None,
                                     "max": max(delays) if delays else None,
                                     "delayed": int(sum(x > 0 for x in delays))},
                  "layer_bumped": sum(1 for x in decisions if x.bump_out or x.bump_back),
                  "wall_s": round(time.time() - t_wall, 2),
                  "decisions": [asdict(x) for x in decisions]}
        return flown, report
