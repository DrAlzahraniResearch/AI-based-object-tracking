"""Milestone 5.5: landing reservations at admission (scheduler terminal="reserve"), keeping a reservation when no
tactical slot certifies, tactical boxes that start at the measured position now, emergency priority (moving
ground departures, displacing reservations), and a short closed loop through the twin."""
import copy
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_sim.hubs import HubParams, HubSequencer  # noqa: E402
from swarm_sim.intents import IntentParams, mission_intents, separated_hv_matrix  # noqa: E402
from swarm_sim.missions import RoutingParams, generate_missions  # noqa: E402
from swarm_sim.osm import Airspace  # noqa: E402
from swarm_sim.scheduler import FcfsScheduler, SchedulerParams  # noqa: E402
from swarm_sim.sim import SimParams  # noqa: E402
from swarm_sim.twin.core import DigitalTwin, TwinParams  # noqa: E402
from swarm_sim.twin.telemetry import PlantParams, TierAPlant  # noqa: E402
from swarm_sim.vehicle import VehicleParams  # noqa: E402

WORLD = ROOT / "data/worlds/mini_town/airspace.json"
RP = RoutingParams(approach_fix_m=100.0)
IP = IntentParams(terminal_lateral_buffer_m=5.0)
H, V = 30_000, 10_000
RES = SchedulerParams(terminal="reserve")


@pytest.fixture(scope="module")
def air():
    if not WORLD.exists():
        pytest.skip("mini_town world not built (scripts/osm_to_world.py)")
    return Airspace.load(WORLD)


@pytest.fixture(scope="module")
def missions(air):
    return generate_missions(air, {"delivery": 60, "emergency": 4, "inspection": 8}, 1800, 1, RP)


def _hub(air, m):
    return next(h for h in air.hubs if h.name == m.hub)


def _inbound(sq, m):
    """Admit m; return (route, a position ~150 m before its fix)."""
    v, dec, _ = sq.admit(m)
    assert v is not None and dec.status == "accepted"
    f = HubSequencer.fix_index(v)
    a, b = v.waypoints[f - 1], v.waypoints[f]
    d = math.hypot(b.x - a.x, b.y - a.y)
    u = max(0.0, 1 - 150.0 / d)
    v.meta = dict(v.meta) | {"pad": list(v.meta["start"])}
    return v, (a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), b.z)


def test_reserve_certifies_the_planned_arrival_and_marks_it(air, missions):
    sq = FcfsScheduler(air, ip=IP, rp=RP, params=RES)
    m = copy.deepcopy(next(x for x in missions if x.kind == "delivery"))
    v, dec, cb = sq.admit(m)
    bl = mission_intents(v, VehicleParams(), IP)
    assert sq.reserved(v) and all(sq.certified(bl, v))
    assert (sq.owner == m.id).sum() == len(bl)
    assert sq.arr[sq.owner == m.id].sum() == sum(b.flow == "arr" for b in bl) > 0
    for flag in ("sequenced", "awaiting", "unreserved"):
        w = copy.deepcopy(v)
        w.meta = dict(w.meta) | {flag: True}
        assert not sq.reserved(w)
    w.meta = dict(v.meta) | {"unreserved": True}
    assert sq.certified(bl, w) == [b.flow != "arr" for b in bl]
    # the sequence mode is unchanged
    sm = FcfsScheduler(air, ip=IP, rp=RP, params=SchedulerParams(terminal="sequence"))
    assert sm.certified(bl, v) == [b.flow != "arr" for b in bl] and not sm.reserved(v)


def test_two_reservations_at_one_hub_are_separated(air, missions):
    sq = FcfsScheduler(air, ip=IP, rp=RP, params=RES)
    hub = missions[0].hub
    ms = [copy.deepcopy(m) for m in missions if m.hub == hub and m.kind == "delivery"][:4]
    for m in ms:
        m.release_t = 100.0
        sq.admit(m)
    keys = sorted(set(sq.owner.tolist()))
    assert len(keys) >= 2
    for i, a in enumerate(keys):            # every pair, landings included
        for b in keys[i + 1:]:
            assert int((~separated_hv_matrix(sq.acc[sq.owner == a], sq.acc[sq.owner == b], H, V)).sum()) == 0


def test_blocked_flight_keeps_its_reservation(air, missions):
    sq = FcfsScheduler(air, ip=IP, rp=RP, params=RES)
    seq = HubSequencer(sq, RP, HubParams(max_hold_s=10.0, pre_holds_s=()))
    m = copy.deepcopy(next(x for x in missions if x.kind == "delivery"))
    r, pos = _inbound(sq, m)
    hub = _hub(air, m)
    t_fix = 1000.0
    before, before_arr = sq.rows(m.id)
    # a forced (uncertified) flight registered after admission blocks the whole terminal area: no tactical slot ...
    sq.register(-8, np.array([[int((hub.x - 300) * 1000), int((hub.x + 300) * 1000), int((hub.y - 300) * 1000),
                               int((hub.y + 300) * 1000), 0, 300_000, int(t_fix * 1000), int((t_fix + 600) * 1000)]],
                             np.int64))
    out = seq.sequence(m.id, r, pos, t_fix, final=True, reserved=True)
    assert out is None and seq.decisions[-1].status == "reserved"      # ... so it keeps the certified reservation
    after, after_arr = sq.rows(m.id)
    assert np.array_equal(after, before) and np.array_equal(after_arr, before_arr)
    assert seq.report()["reserved"] == 1 and seq.report()["contingent"] == 0
    # without a reservation the same flight would go contingent
    v, status, _ = seq.sequence(m.id, r, pos, t_fix + 2, final=True, reserved=False)
    assert status == "contingent"


def test_tactical_boxes_start_now(air, missions):
    m = copy.deepcopy(next(x for x in missions if x.kind == "delivery"))
    now = 1000.0
    for mode, starts_now in (("reserve", True), ("sequence", False)):
        sq = FcfsScheduler(air, ip=IP, rp=RP, params=SchedulerParams(terminal=mode))
        seq = HubSequencer(sq, RP)
        r, pos = _inbound(sq, copy.deepcopy(m))
        hub = _hub(air, m)
        v = seq._route(r, pos, now, (hub.x + 100.0, hub.y, 25.0), r.pad, 0.0, pos[2])
        cb = seq._cb(v, None, now)
        assert (cb[:, 6].min() >= now * 1000) == starts_now


def test_eta_ends_at_the_fix_for_a_reserved_flight(air, missions):
    m = copy.deepcopy(next(x for x in missions if x.kind == "delivery"))
    for mode, past in (("reserve", None), ("sequence", 0.0)):
        sq = FcfsScheduler(air, ip=IP, rp=RP, params=SchedulerParams(terminal=mode))
        seq = HubSequencer(sq, RP)
        v, _ = _inbound(sq, copy.deepcopy(m))
        f = seq.fix_index(v)
        assert seq.eta(v, f + 1, (0, 0, 0), (5, 0, 0)) == past
        assert seq.eta(v, f, (v.waypoints[f].x + 50, v.waypoints[f].y, 0), (5, 0, 0)) == pytest.approx(10.0)


def test_priority_displaces_reservations_and_ground_rows_only(air, missions):
    sq = FcfsScheduler(air, ip=IP, rp=RP, params=RES)
    ms = [copy.deepcopy(m) for m in missions if m.kind == "delivery"][:2]
    for m in ms:
        sq.admit(m)
    a, b = ms[0].id, ms[1].id
    mask = sq.displaceable(ground=frozenset({a}), soft=frozenset({b}))
    assert np.array_equal(mask, (sq.owner == a) | ((sq.owner == b) & sq.arr))
    n_b, n_arr = (sq.owner == b).sum(), ((sq.owner == b) & sq.arr).sum()
    sq.unreserve(b)
    assert (sq.owner == b).sum() == n_b - n_arr and not sq.arr[sq.owner == b].any()
    # admission that ignores displaceable rows
    e = copy.deepcopy(ms[0])
    e.id = 999
    sq.withdraw(a)
    sq.register(a, sq._boxes_arr(e)[0])         # the same plan held by another flight: e conflicts with it ...
    v, dec, _ = sq.admit(e, ignore=sq.owner == a)
    assert dec.status == "accepted"            # ... unless that flight's rows may be moved


def test_twin_with_landing_reservations(air):
    sp = SimParams(duration_s=600, max_extra_s=900)
    vp = VehicleParams()
    req = generate_missions(air, {"delivery": 60, "emergency": 4, "inspection": 8}, 600, 3, RP)
    sched = FcfsScheduler(air, vp, IP, RP, SchedulerParams(terminal="reserve", priority_max_delay_s=30.0))
    seq = HubSequencer(sched, RP)
    plant = TierAPlant(air, req, vp, sp, None, PlantParams(seed=2))
    tw = DigitalTwin(air, plant, req, sched, vp, IP, sp, lambda dt: None, TwinParams(predict_every_s=60), None, None,
                     echo=lambda *_: None, sequencer=seq)
    rep = tw.run(sp.duration_s + sp.max_extra_s)
    res = plant.sim.result()
    flown = sum(1 for d in tw.decisions if d["status"] != "rejected" and not d.get("rescheduled"))
    assert sum(s["state"] == "landed" for s in res.status) == flown
    h = rep["hubs"]
    assert h["arrivals"] == flown and h["sequenced"] + h["reserved"] + h["contingent"] == flown
    uncert = {d["mission"] for d in tw.decisions if d["status"] == "accepted_uncertified"}
    assert h["contingent"] <= len(uncert)       # only flights launched without a certified plan
    certified = {"accepted", "replanned", "sequenced", "awaiting"}

    def ok_at(vid, t):
        v = next((v for v in tw.intent_versions if v["vid"] == vid and v["valid_from"] <= t <= v["valid_to"]), None)
        return v is not None and v["status"] in certified and all(v["cert"])

    for e in res.los_events:          # every loss of separation involves a flight nobody certified at the time
        assert not (ok_at(e["a"], e["t_start"]) and ok_at(e["b"], e["t_start"]))
    assert rep["conformance"]["rate"] > 0.99


def test_reserve_fallback_admits_without_a_reservation(air, missions):
    m = copy.deepcopy(next(x for x in missions if x.kind == "delivery"))
    hub = _hub(air, m)
    for fb, expect in ((False, "rejected"), (True, "accepted")):
        sq = FcfsScheduler(air, ip=IP, rp=RP, params=SchedulerParams(terminal="reserve", max_delay_s=0.0,
                                                                     reserve_fallback=fb))
        # the pads are booked from a minute after take-off: only a plan without a landing reservation fits
        sq.register(-1, np.array([[int((hub.x - 60) * 1000), int((hub.x + 60) * 1000), int((hub.y - 60) * 1000),
                                   int((hub.y + 60) * 1000), 0, 60_000, int((m.release_t + 60) * 1000),
                                   10_000_000]], np.int64))
        v, dec, _ = sq.admit(m)
        assert dec.status == expect
        if fb:
            assert v.meta.get("unreserved") and not sq.reserved(v) and not sq.arr[sq.owner == m.id].any()
