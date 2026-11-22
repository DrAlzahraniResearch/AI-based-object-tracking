"""The scene of a twin run at any time t: what every viewer draws.

A `Scene` is fed twin-log messages (`feed(topic, t, msg)`), either all at once from a finished run
(`Scene.from_run`) or one by one from a live twin (scripts/gz_view.py --live). `frame(t)` returns:

* vehicles: the twin's estimates (`/twin/state`, 1 Hz), linearly interpolated between samples, with conformance
* truth: the plant's true positions (`/plant/truth`, sim plants only), interpolated the same way
* predicted: the latest `/twin/prediction` (positions at +10/30/60 s)
* intents: the intent boxes in force at t (every plan version valid at t, every box whose window contains t)
* events: alerts, replans, decisions, calibration and arrivals since the previous frame

Intent versions come from the run's intents.json, or live from `/twin/intent_version` (swarm_sim/gz/live.py).
"""
from __future__ import annotations

import bisect
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..osm import Airspace

EVENT_TOPICS = ("/twin/alert", "/twin/replan", "/twin/calibration", "/twin/decision", "/twin/arrival")
INTENT_TOPIC = "/twin/intent_version"


@dataclass
class VehicleView:
    vid: int
    pos: np.ndarray
    vel: np.ndarray
    conforming: bool = True


@dataclass
class IntentView:
    vid: int
    lo: np.ndarray            # min corner, m
    size: np.ndarray          # extent, m
    certified: bool
    phase: str                # en_route | terminal
    status: str               # plan version status (admitted, replanned, contingent, ...)

    @property
    def center(self) -> np.ndarray:
        return self.lo + self.size / 2


@dataclass
class Event:
    t: float
    kind: str                 # alert | replan | calibration | decision | arrival
    text: str
    warn: bool = False


@dataclass
class Frame:
    t: float
    vehicles: dict[int, VehicleView] = field(default_factory=dict)
    truth: dict[int, np.ndarray] = field(default_factory=dict)
    predicted: dict[float, dict[int, np.ndarray]] = field(default_factory=dict)   # horizon s -> vid -> pos
    intents: list[IntentView] = field(default_factory=list)
    events: list[Event] = field(default_factory=list)

    @property
    def nonconforming(self) -> int:
        return sum(not v.conforming for v in self.vehicles.values())


class _Tracks:
    """Per-vehicle time series, appended in time order; sampled with linear interpolation."""

    def __init__(self, max_gap_s: float = 2.5, hold_s: float = 1.5):
        self.t: dict[int, list] = {}
        self.p: dict[int, list] = {}
        self.c: dict[int, list] = {}
        self.max_gap, self.hold = max_gap_s, hold_s

    def add(self, vid: int, t: float, pos, flag: bool = True):
        ts = self.t.setdefault(vid, [])
        if ts and t < ts[-1]:
            return                                   # out of order (live): drop
        ts.append(t)
        self.p.setdefault(vid, []).append(np.asarray(pos, float))
        self.c.setdefault(vid, []).append(bool(flag))

    def sample(self, t: float) -> dict[int, tuple]:
        out = {}
        for vid, ts in self.t.items():
            i = bisect.bisect_right(ts, t) - 1
            if i < 0:
                continue
            p0, c0 = self.p[vid][i], self.c[vid][i]
            if i + 1 < len(ts) and ts[i + 1] - ts[i] <= self.max_gap:
                a = (t - ts[i]) / (ts[i + 1] - ts[i])
                p1 = self.p[vid][i + 1]
                out[vid] = (p0 + a * (p1 - p0), (p1 - p0) / (ts[i + 1] - ts[i]), c0)
            elif t - ts[i] <= self.hold:
                v = ((p0 - self.p[vid][i - 1]) / (ts[i] - ts[i - 1])
                     if i > 0 and 0 < ts[i] - ts[i - 1] <= self.max_gap else np.zeros(3))
                out[vid] = (p0, v, c0)
        return out

    def span(self) -> tuple[float, float]:
        firsts = [ts[0] for ts in self.t.values() if ts]
        lasts = [ts[-1] for ts in self.t.values() if ts]
        return (min(firsts), max(lasts)) if firsts else (0.0, 0.0)


def _event_text(topic: str, msg: dict) -> tuple[str, bool]:
    kind = topic.rsplit("/", 1)[-1]
    if kind == "alert":
        detail = msg.get("event_detail")
        return (f"vehicle {msg.get('vid')}: {msg.get('event')}{f' ({detail})' if detail else ''}",
                msg.get("event") == "nonconforming")
    if kind == "replan":
        return f"vehicle {msg.get('vid')} replanned: {msg.get('status')} (hover {msg.get('hover_s', 0)} s)", \
            msg.get("status") == "contingent"
    if kind == "decision":
        st = msg.get("status")
        return f"request {msg.get('mission')} ({msg.get('kind')}): {st}, delay {msg.get('ground_delay_s', 0)} s", \
            st in ("rejected", "accepted_uncertified")
    if kind == "arrival":
        return (f"vehicle {msg.get('vid')} arrival at {msg.get('hub')}: {msg.get('status')}, hold {msg.get('hold_s', 0)} s",
                msg.get("status") == "contingent")
    return f"{kind}: {json.dumps(msg, default=str)[:160]}", False


class Scene:
    def __init__(self, air: Airspace | None = None, versions: list | None = None, state_hz: float = 1.0):
        self.air = air
        gap = 2.5 / state_hz
        self.est = _Tracks(gap, 1.5 / state_hz)
        self.tru = _Tracks(gap, 1.5 / state_hz)
        self.preds: list[tuple[float, dict]] = []
        self.events: list[Event] = []
        self.versions: dict[tuple, dict] = {}
        for v in versions or []:
            self._upsert(v)
        self._last_t = -math.inf

    # ------------------------------------------------------------------ input
    def _upsert(self, v: dict):
        self.versions[(v["vid"], v["valid_from"])] = v

    def feed(self, topic: str, t: float, msg: dict):
        if topic == "/twin/state":
            for k, v in msg["vehicles"].items():
                self.est.add(int(k), t, v["pos"], v.get("conforming", True))
        elif topic == "/plant/truth":
            for k, p in msg["vehicles"].items():
                self.tru.add(int(k), t, p)
        elif topic == "/twin/prediction":
            pos = {float(h): {int(k): np.asarray(p, float) for k, p in d.items()} for h, d in msg.get("pos", {}).items()}
            self.preds.append((t, pos))
        elif topic == INTENT_TOPIC:
            self._upsert(msg)
        elif topic in EVENT_TOPICS:
            text, warn = _event_text(topic, msg)
            self.events.append(Event(t, topic.rsplit("/", 1)[-1], text, warn))

    @classmethod
    def from_run(cls, run: str | Path, world_dir: str | Path | None = None) -> "Scene":
        from ..twin.log import read_log
        run = Path(run)
        rep = json.loads((run / "report.json").read_text())
        world_dir = Path(world_dir) if world_dir else Path(__file__).resolve().parents[2] / "data/worlds"
        air = Airspace.load(world_dir / rep["world"] / "airspace.json")
        vers = json.loads((run / "intents.json").read_text())["versions"]
        hz = ((rep.get("params") or {}).get("twin") or {}).get("log_state_hz", 1.0)
        sc = cls(air, vers, hz)
        logf = run / "twin_log.mcap"
        if not logf.exists():
            logf = run / "twin_log.jsonl"
        for topic, t, msg in read_log(logf, ["/twin/state", "/plant/truth", "/twin/prediction", *EVENT_TOPICS]):
            sc.feed(topic, t, msg)
        sc.events.sort(key=lambda e: e.t)
        return sc

    # ------------------------------------------------------------------ output
    def span(self) -> tuple[float, float]:
        return self.est.span()

    def intents_at(self, t: float) -> list[IntentView]:
        out, tm = [], t * 1000.0
        for v in self.versions.values():
            vt = v.get("valid_to")
            if not (v["valid_from"] <= t <= (math.inf if vt is None else vt)):
                continue
            cert = v.get("cert") or [True] * len(v["int"])
            phase = v.get("phase") or ["en_route"] * len(v["int"])
            for b, c, ph in zip(v["int"], cert, phase):
                if b[6] <= tm <= b[7]:
                    lo = np.array([b[0], b[2], b[4]], float) / 1000
                    size = np.array([b[1] - b[0], b[3] - b[2], b[5] - b[4]], float) / 1000
                    out.append(IntentView(int(v["vid"]), lo, size, bool(c), ph, v.get("status", "")))
        return out

    def frame(self, t: float, since: float | None = None) -> Frame:
        """The scene at t; events in (since, t] (default: since the previous call)."""
        since = self._last_t if since is None else since
        self._last_t = t
        f = Frame(t)
        for vid, (p, v, ok) in self.est.sample(t).items():
            f.vehicles[vid] = VehicleView(vid, p, v, ok)
        f.truth = {vid: p for vid, (p, _v, _c) in self.tru.sample(t).items()}
        i = bisect.bisect_right([p[0] for p in self.preds], t) - 1
        if i >= 0 and t - self.preds[i][0] <= 15.0:
            f.predicted = self.preds[i][1]
        f.intents = self.intents_at(t)
        lo = bisect.bisect_right([e.t for e in self.events], since)
        hi = bisect.bisect_right([e.t for e in self.events], t)
        f.events = self.events[lo:hi]
        return f

    def times(self, every: float, start: float | None = None, end: float | None = None):
        t0, t1 = self.span()
        t0 = t0 if start is None else max(start, t0)
        t1 = t1 if end is None else min(end, t1)
        n = int(math.floor((t1 - t0) / every + 1e-9)) + 1 if t1 >= t0 else 0
        return [t0 + k * every for k in range(n)]


# ------------------------------------------------------------------ static world geometry
def world_mesh(air: Airspace) -> tuple[np.ndarray, np.ndarray]:
    """Buildings as one triangle mesh (walls + triangulated roofs): vertices (n, 3), triangles (m, 3)."""
    from shapely.geometry import Point, Polygon
    from shapely.ops import triangulate
    verts, tris = [], []
    for b in air.buildings:
        poly = Polygon(b.polygon)
        if not poly.is_valid or poly.area < 1:
            continue
        ring = list(poly.exterior.coords)[:-1]
        base = len(verts)
        for x, y in ring:
            verts += [(x, y, b.min_height), (x, y, b.height)]
        n = len(ring)
        for i in range(n):
            j = (i + 1) % n
            a0, a1, b0, b1 = base + 2 * i, base + 2 * i + 1, base + 2 * j, base + 2 * j + 1
            tris += [(a0, b0, b1), (a0, b1, a1)]
        for tr in triangulate(poly):
            if poly.contains(Point(tr.centroid.x, tr.centroid.y)):
                k = len(verts)
                verts += [(x, y, b.height) for x, y in list(tr.exterior.coords)[:3]]
                tris.append((k, k + 1, k + 2))
    return np.array(verts, float).reshape(-1, 3), np.array(tris, int).reshape(-1, 3)


VEHICLE_PALETTE = np.array([[31, 119, 180], [255, 127, 14], [44, 160, 44], [148, 103, 189], [140, 86, 75],
                            [227, 119, 194], [188, 189, 34], [23, 190, 207]])
OK_RGB = (40, 180, 80)
NC_RGB = (230, 40, 40)
TRUTH_RGB = (120, 120, 120)
PRED_RGB = (250, 200, 60)
