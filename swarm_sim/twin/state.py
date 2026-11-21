"""Twin state: per-vehicle estimates from noisy, late telemetry, and the monitors that run on them.

* `TwinState`          - alpha-beta filter per vehicle; `estimate(t)` extrapolates over the link latency.
* `ConformanceMonitor` - the online counterpart of the offline audit: is each vehicle inside one of its
                         declared intent boxes right now? Uses the Lean-verified `inside` when the
                         checker library is available, else the Python reference (`inside_ref`).
* `SpeedCalibrator`    - estimates the fleet's effective cruise / climb / descent speeds from telemetry
                         and reports drift from the planning model (the twin re-calibrates itself).
"""
from __future__ import annotations

import math
from collections import defaultdict, deque
from dataclasses import dataclass, replace

import numpy as np

from ..intents import Box4, inside_ref
from ..vehicle import VehicleParams
from .telemetry import Telemetry


@dataclass
class Track:
    t: float
    pos: np.ndarray
    vel: np.ndarray
    state: str
    wp_idx: int
    hold_until: float
    n: int = 0
    route_ver: int = 1


class TwinState:
    def __init__(self, alpha: float = 0.5, beta: float = 0.5):
        self.alpha, self.beta = alpha, beta              # position gain, velocity low-pass gain
        self.tracks: dict[int, Track] = {}

    def ingest(self, z: Telemetry):
        tr = self.tracks.get(z.vid)
        if tr is None or z.state != "flying" or tr.state != "flying":
            self.tracks[z.vid] = Track(z.t, np.array(z.pos, float), np.array(z.vel, float), z.state, z.wp_idx,
                                       z.hold_until, 1, z.route_ver)
            return
        dt = z.t - tr.t
        if dt <= 0:
            return                                           # out of order / duplicate
        pred = tr.pos + tr.vel * dt
        r = np.asarray(z.pos) - pred
        tr.pos = pred + self.alpha * r
        tr.vel = self.beta * np.asarray(z.vel) + (1 - self.beta) * tr.vel
        tr.t, tr.n = z.t, tr.n + 1
        if z.route_ver >= tr.route_ver:                      # progress of an older route is stale after a replan
            tr.wp_idx, tr.hold_until, tr.route_ver = z.wp_idx, z.hold_until, z.route_ver

    def rerouted(self, vid: int):
        """The twin just replaced this vehicle's route: progress restarts at waypoint 0 of the next version."""
        tr = self.tracks[vid]
        tr.wp_idx, tr.hold_until, tr.route_ver = 0, -1.0, tr.route_ver + 1

    def flying(self) -> list[int]:
        return [k for k, tr in self.tracks.items() if tr.state == "flying"]

    def estimate(self, vid: int, t: float) -> np.ndarray:
        tr = self.tracks[vid]
        if tr.hold_until >= 0:
            return tr.pos.copy()                             # station-keeping: do not extrapolate noise
        return tr.pos + tr.vel * max(0.0, t - tr.t)


class ConformanceMonitor:
    """Tracks, per vehicle, whether its estimated position is inside any of its current intent boxes.

    A vehicle becomes non-conforming after `confirm_s` of continuous misses (filters noise at box edges)
    and conforming again after `clear_s` of hits. Transitions are returned as alerts."""

    def __init__(self, checker=None, margin_mm: int = 0, confirm_s: float = 2.0, clear_s: float = 2.0):
        self.checker = checker
        self.margin = margin_mm
        self.confirm_s, self.clear_s = confirm_s, clear_s
        self.boxes: dict[int, np.ndarray] = {}
        self.phase: dict[int, list] = {}
        self.ok: dict[int, bool] = {}
        self.since: dict[int, float] = {}
        self.samples = defaultdict(lambda: [0, 0])          # vid -> [inside, total]
        self.by_phase = defaultdict(lambda: [0, 0])          # phase of the box it should be in -> [inside, total]

    def set_intents(self, vid: int, boxes: list[Box4]):
        self.boxes[vid] = np.array([b.as_int() for b in boxes], np.int64).reshape(-1, 8)
        self.phase[vid] = [b.phase for b in boxes]
        self.ok.setdefault(vid, True)
        self.since.pop(vid, None)

    def drop(self, vid: int):
        for d in (self.boxes, self.phase, self.ok, self.since):
            d.pop(vid, None)

    def _inside(self, B: np.ndarray, x, y, z, t) -> int:
        """Index of a box containing the sample, or -1. Time-filtered in numpy, decided by the checker."""
        cand = np.flatnonzero((B[:, 6] <= t) & (t <= B[:, 7]))
        for k in cand:
            b = B[k]
            hit = (self.checker.inside(b, self.margin, x, y, z, t) if self.checker is not None
                   else inside_ref(tuple(int(v) for v in b), self.margin, x, y, z, t))
            if hit:
                return int(k)
        return -1

    def check(self, vid: int, pos: np.ndarray, t: float) -> dict | None:
        B = self.boxes.get(vid)
        if B is None or len(B) == 0:
            return None
        x, y, z = (int(round(v * 1000)) for v in (pos[0], pos[1], max(pos[2], 0.0)))   # an estimate below ground is on it
        tm = int(round(t * 1000))
        k = self._inside(B, x, y, z, tm)
        hit = k >= 0
        live = np.flatnonzero((B[:, 6] <= tm) & (tm <= B[:, 7]))
        phase = (self.phase[vid][k] if hit else
                 self.phase[vid][int(live[0])] if len(live) else ("late" if tm > B[:, 7].max() else "early"))
        s = self.samples[vid]
        s[0] += hit
        s[1] += 1
        bp = self.by_phase[phase]
        bp[0] += hit
        bp[1] += 1
        if hit == self.ok[vid]:
            self.since.pop(vid, None)
            return None
        t0 = self.since.setdefault(vid, t)
        if t - t0 >= (self.clear_s if hit else self.confirm_s):
            self.ok[vid] = hit
            self.since.pop(vid, None)
            return {"t": round(t, 2), "vid": vid, "event": "conforming" if hit else "nonconforming",
                    "phase": phase, "pos": [round(float(v), 2) for v in pos]}
        return None

    def summary(self) -> dict:
        tot = np.array(list(self.samples.values()) or [[0, 0]]).sum(axis=0)
        return {"samples": int(tot[1]), "inside": int(tot[0]),
                "rate": round(float(tot[0] / tot[1]), 4) if tot[1] else None,
                "by_phase": {k: {"inside": v[0], "samples": v[1], "rate": round(v[0] / v[1], 4) if v[1] else None}
                             for k, v in sorted(self.by_phase.items())},
                "checker": "verified" if self.checker is not None else "python-reference"}


class SpeedCalibrator:
    """Measured cruise / climb / descent speeds vs the planning model.

    Per vehicle: recent ground speed on cruise legs away from both ends (steady state) and vertical speed
    on near-vertical legs. Fleet: the median over vehicles of each vehicle's median (one vote per
    vehicle, so one degraded vehicle cannot drag the fleet model), from the `fleet_window` most recently
    seen vehicles, at least `min_vehicles` of them. A fleet update is proposed when a speed drifts more
    than `tol` from the current model."""

    def __init__(self, vp: VehicleParams, min_samples: int = 10, min_vehicles: int = 3, fleet_window: int = 20,
                 tol: float = 0.05, min_n: int | None = None, cfg: VehicleParams | None = None):
        self.vp0 = vp                       # the model the twin started from (config, or a prior session's)
        self.cfg = cfg or vp                # the vehicles' configuration: what the guidance commands
        self.vp = vp
        self.min_samples = min_n or min_samples
        self.min_vehicles, self.fleet_window, self.tol = min_vehicles, fleet_window, tol
        self.q = {k: defaultdict(lambda: deque(maxlen=50)) for k in ("h", "up", "dn")}
        self.recent = {k: deque(maxlen=fleet_window) for k in ("h", "up", "dn")}
        self.updates: list = []

    def _add(self, kind, vid, v):
        self.q[kind][vid].append(v)
        r = self.recent[kind]
        if vid in r:
            r.remove(vid)
        r.append(vid)

    def observe(self, tr: Track, leg_a, leg_b, vid: int = -1):
        if tr.hold_until >= 0 or leg_a is None:
            return
        a, b, p = np.asarray(leg_a), np.asarray(leg_b), tr.pos
        dxy = math.hypot(*(b[:2] - a[:2]))
        dz = b[2] - a[2]
        if dxy > 150 and abs(dz) < 1:
            u = (p[:2] - a[:2]) @ (b[:2] - a[:2]) / dxy ** 2
            if 0.2 < u < 0.8:
                self._add("h", vid, float(np.hypot(*tr.vel[:2])))
        elif dxy < 1 and abs(dz) > 15:
            u = (p[2] - a[2]) / dz
            if 0.3 < u < 0.7:
                self._add("up" if dz > 0 else "dn", vid, abs(float(tr.vel[2])))

    def vehicle_speed(self, vid: int, min_n: int = 10) -> float | None:
        """This vehicle's recent cruise ground speed (median of its last 25 samples), if enough samples."""
        q = self.q["h"].get(vid)
        return float(np.median(list(q)[-25:])) if q is not None and len(q) >= min_n else None

    def fleet(self, kind: str) -> tuple[float | None, int]:
        meds = [float(np.median(self.q[kind][v])) for v in self.recent[kind] if len(self.q[kind][v]) >= self.min_samples]
        return (float(np.median(meds)) if len(meds) >= self.min_vehicles else None), len(meds)

    def maybe_update(self, t: float) -> VehicleParams | None:
        new = {}
        for kind, key in (("h", "cruise_xy"), ("up", "v_up_max"), ("dn", "v_down_max")):
            med, _n = self.fleet(kind)
            cur = getattr(self.vp, key)
            if med is not None and abs(med - cur) / cur > self.tol:
                new[key] = round(med, 2)
        if not new:
            return None
        if "cruise_xy" in new:          # slower than commanded: the vehicles are speed-limited, so cap the model too
            new["v_xy_max"] = min(self.cfg.v_xy_max, new["cruise_xy"]) if new["cruise_xy"] < self.cfg.cruise_xy \
                else self.cfg.v_xy_max
        self.vp = replace(self.vp, **new)
        self.updates.append({"t": round(t, 2), **new, "vehicles": {k: self.fleet(k)[1] for k in self.q}})
        return self.vp
