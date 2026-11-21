"""Run metrics: completion, delay, loss of separation, intrusions, intents and conformance."""
from __future__ import annotations

import itertools
import math

import numpy as np

from .intents import IntentParams, inside_ref, mission_intents, planned_schedule, separated_hv_matrix, separated_hv_ref
from .vehicle import VehicleParams


def strategic_conflicts(intents_by_mission: dict, h_mm: int, v_mm: int, separated=separated_hv_ref) -> list:
    """Pairs of missions whose (en-route) intents are NOT cylinder-separated (Theorem 2 checker = False
    for some box pair)."""
    out = []
    ids = sorted(intents_by_mission)
    for a, b in itertools.combinations(ids, 2):
        A, B = intents_by_mission[a], intents_by_mission[b]
        if not A or not B:
            continue
        # cheap prefilter on overall time span
        if max(x[7] for x in A) < min(x[6] for x in B) or max(x[7] for x in B) < min(x[6] for x in A):
            continue
        if separated is separated_hv_ref:
            n_bad = int((~separated_hv_matrix(A, B, h_mm, v_mm)).sum())
        else:
            n_bad = sum(1 for x in A for y in B if not separated(x, y, h_mm, v_mm))
        if n_bad:
            out.append({"a": a, "b": b, "box_pairs_not_separated": n_bad})
    return out


def conformance(result, intents_by_mission: dict, margin_mm: int = 0, inside=inside_ref) -> dict:
    """Fraction of logged airborne samples that lie inside some volume of their own mission's intent."""
    ok = tot = 0
    worst = []
    for i, m in enumerate(result.missions):
        boxes = intents_by_mission[m.id]
        for k, t in enumerate(result.times):
            p = result.tracks[k, i]
            if np.isnan(p[0]) or p[2] < 0.5:
                continue
            xi, yi, zi, ti = (int(round(p[0] * 1000)), int(round(p[1] * 1000)), int(round(p[2] * 1000)),
                              int(round(t * 1000)))
            tot += 1
            if any(inside(b, margin_mm, xi, yi, zi, ti) for b in boxes):
                ok += 1
            elif len(worst) < 20:
                worst.append({"mission": m.id, "t": round(float(t), 2), "pos": [round(float(v), 2) for v in p]})
    return {"samples": tot, "conforming": ok, "rate": ok / tot if tot else float("nan"), "first_violations": worst}


def summarise(result, vp: VehicleParams = VehicleParams(), ip: IntentParams = IntentParams(),
              h_m: float = 30.0, v_m: float = 10.0, separated=separated_hv_ref, inside=inside_ref) -> dict:
    st = result.status
    done = [s for s in st if s["state"] == "landed"]
    delays, durations = [], []
    for s, m in zip(st, result.missions):
        if s["landed_t"] is None:
            continue
        plan = planned_schedule(m, vp)
        planned_end = plan[-1][3]
        delays.append(s["landed_t"] - planned_end)
        durations.append(s["landed_t"] - s["takeoff_t"])
    flight_h = sum(durations) / 3600.0 if durations else 0.0
    los_en = [e for e in result.los_events if not e["terminal"]]
    los_term = [e for e in result.los_events if e["terminal"]]
    boxes = {m.id: mission_intents(m, vp, ip) for m in result.missions}
    intents = {k: [b.as_int() for b in v] for k, v in boxes.items()}
    en_route = {k: [b.as_int() for b in v if b.phase == "en_route"] for k, v in boxes.items()}
    sc = strategic_conflicts(en_route, int(round(h_m * 1000)), int(round(v_m * 1000)), separated)
    conf = conformance(result, intents, 0, inside)
    ground = [m.meta.get("ground_delay_s", 0.0) for m in result.missions]
    by_kind = {}
    for s in st:
        by_kind.setdefault(s["kind"], [0, 0])
        by_kind[s["kind"]][0] += 1
        by_kind[s["kind"]][1] += s["state"] == "landed"
    return {
        "missions": len(st), "completed": len(done),
        "by_kind": {k: {"missions": v[0], "completed": v[1]} for k, v in by_kind.items()},
        "flight_hours": round(flight_h, 3),
        "delay_s_vs_plan": {"median": _q(delays, 50), "p95": _q(delays, 95), "max": _q(delays, 100)},
        "los_events": {"en_route": len(los_en), "terminal": len(los_term),
                       "en_route_per_flight_hour": round(len(los_en) / flight_h, 3) if flight_h else None,
                       "min_dist_m": round(min((e["min_dist_m"] for e in result.los_events), default=math.inf), 2)},
        "intrusions": {k: len(v) for k, v in result.intrusions.items()},
        "strategic_conflicts": {"mission_pairs": len(sc), "phase": "en_route", "h_m": h_m, "v_m": v_m},
        "ground_delay_s": {"median": _q(ground, 50), "p95": _q(ground, 95), "max": _q(ground, 100),
                           "delayed": int(sum(g > 0 for g in ground))},
        "conformance": {k: v for k, v in conf.items() if k != "first_violations"},
        "wall_s": round(result.wall_s, 2),
    }


def _q(v, q):
    return round(float(np.percentile(v, q)), 2) if v else None
