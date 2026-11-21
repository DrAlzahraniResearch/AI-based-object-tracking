"""Milestone 5: approach-fix routing, intent flows and terminal buffers, terminal certification in the scheduler,
the hub sequencer (holding, pads, arrival priority), and a short closed loop through the twin."""
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
from swarm_sim.missions import RoutingParams, approach_fix, generate_missions  # noqa: E402
from swarm_sim.osm import Airspace  # noqa: E402
from swarm_sim.scheduler import FcfsScheduler, SchedulerParams, variant, with_pad  # noqa: E402
from swarm_sim.sim import SimParams  # noqa: E402
from swarm_sim.twin.core import DigitalTwin, TwinParams  # noqa: E402
from swarm_sim.twin.telemetry import PlantParams, TierAPlant  # noqa: E402
from swarm_sim.vehicle import VehicleParams  # noqa: E402

WORLD = ROOT / "data/worlds/mini_town/airspace.json"
RP = RoutingParams(approach_fix_m=70.0)
IP = IntentParams(terminal_lateral_buffer_m=5.0)
H, V = 30_000, 10_000


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


# ------------------------------------------------------------------ routing and intents
def test_routes_end_at_an_approach_fix_on_the_inbound_bearing(air, missions):
    for m in missions[:20]:
        tags = [w.tag for w in m.waypoints]
        assert tags[-3:] == ["fix", "approach", "land"]
        hub = _hub(air, m)
        f = m.waypoints[-3]
        assert abs(math.hypot(f.x - hub.x, f.y - hub.y) - 70.0) < 1e-6
        last_target = m.waypoints[-5]
        assert abs(math.atan2(f.y - hub.y, f.x - hub.x) - math.atan2(last_target.y - hub.y, last_target.x - hub.x)) < 1e-6
    assert approach_fix((0, 0), (100, 0), 70) == (70.0, 0.0)


def test_default_routing_is_unchanged(air):
    ms = generate_missions(air, {"delivery": 60}, 600, 1, RoutingParams())
    assert all([w.tag for w in m.waypoints][-3:] == ["climb", "cruise", "land"] for m in ms)


def test_intent_flows_and_terminal_buffer(missions):
    m = next(x for x in missions if x.kind == "delivery")
    bl = mission_intents(m, VehicleParams(), IP)
    flows = [b.flow for b in bl]
    assert flows[0] == "dep" and flows[-1] == "arr" and "" in flows
    first_arr = flows.index("arr")
    assert all(f == "arr" for f in flows[first_arr:])          # everything after the fix
    assert all(f == "dep" for f in flows[:flows.index("")])     # leading terminal boxes
    for b in bl:
        width = b.x1 - b.x0
        if b.phase == "terminal" and width < 30:                # a vertical slice in a pad column: 2 x 5 m
            assert abs(width - 10.0) < 1e-6
    wide = mission_intents(m, VehicleParams(), IntentParams())   # default: 20 m everywhere
    assert min(b.x1 - b.x0 for b in wide) >= 40.0 - 1e-6


def test_variant_bumps_the_return_layer_through_the_fix(air, missions):
    m = next(x for x in missions if x.kind == "delivery")
    v = variant(m, air, RP, 10.0, 0, 1)
    if v is None:
        pytest.skip("ceiling")
    dz = [b.z - a.z for a, b in zip(m.waypoints, v.waypoints)]
    tags = [w.tag for w in m.waypoints]
    back = max(i for i, t in enumerate(tags) if t == "climb")
    assert all(abs(dz[i] - 30.0) < 1e-9 for i in range(back, len(tags) - 1)) and dz[-1] == 0.0
    assert v.release_t == m.release_t + 10.0


def test_with_pad_moves_departure_and_default_landing(air, missions):
    m = next(x for x in missions if x.kind == "delivery")
    hub = _hub(air, m)
    p = (m.pad + 1) % hub.pads
    v = with_pad(m, air, RP, p)
    if v is None:
        pytest.skip("pad legs not clear")
    px, py = hub.pad_positions()[p]
    assert v.pad == p and v.meta["start"][:2] == [px, py]
    assert (v.waypoints[0].x, v.waypoints[0].y) == (px, py) and (v.waypoints[-1].x, v.waypoints[-1].y) == (px, py)


# ------------------------------------------------------------------ scheduler
def test_certified_masks(air, missions):
    m = next(x for x in missions if x.kind == "delivery")
    bl = mission_intents(m, VehicleParams(), IP)
    ex = FcfsScheduler(air, ip=IP, rp=RP)
    sq = FcfsScheduler(air, ip=IP, rp=RP, params=SchedulerParams(terminal="sequence"))
    assert ex.certified(bl, m) == [b.phase == "en_route" for b in bl]
    assert sq.certified(bl, m) == [b.flow != "arr" for b in bl]
    m2 = copy.deepcopy(m)
    m2.meta = dict(m2.meta) | {"sequenced": True}
    assert all(sq.certified(bl, m2))


def test_sequence_mode_certifies_departure_climbs(air, missions):
    sq = FcfsScheduler(air, ip=IP, rp=RP, params=SchedulerParams(terminal="sequence"))
    same_hub = [m for m in missions if m.hub == missions[0].hub][:6]
    for m in same_hub:
        m = copy.deepcopy(m)
        m.release_t = 100.0                                     # all want to leave at once
        v, dec, cb = sq.admit(m)
        assert dec.status in ("accepted", "accepted_uncertified", "rejected")
    acc, own = sq.acc, sq.owner
    for a in set(own.tolist()):                                 # every pair of flights, terminal boxes included
        for b in set(own.tolist()):
            if a < b:
                assert int((~separated_hv_matrix(acc[own == a], acc[own == b], H, V)).sum()) == 0 or \
                    any(d.status == "accepted_uncertified" for d in [])


# ------------------------------------------------------------------ hub sequencer
def _inbound(air, m, sq):
    """Admit m, then return (route, a position 15 s before its fix)."""
    v, dec, _ = sq.admit(m)
    assert v is not None
    f = HubSequencer.fix_index(v)
    a, b = v.waypoints[f - 1], v.waypoints[f]
    d = math.hypot(b.x - a.x, b.y - a.y)
    u = max(0.0, 1 - 150.0 / d)
    pos = (a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), b.z)
    v.meta = dict(v.meta) | {"pad": list(v.meta["start"])}
    return v, pos


def test_sequencer_holds_the_second_of_two_simultaneous_arrivals(air, missions):
    sq = FcfsScheduler(air, ip=IP, rp=RP, params=SchedulerParams(terminal="sequence"))
    seq = HubSequencer(sq, RP, HubParams())
    hub = missions[0].hub
    ms = [copy.deepcopy(m) for m in missions if m.hub == hub and m.kind == "delivery"][:2]
    r1, p1 = _inbound(air, ms[0], sq)
    r2, p2 = _inbound(air, ms[1], sq)
    now = 1000.0
    out1 = seq.sequence(ms[0].id, r1, p1, now)
    out2 = seq.sequence(ms[1].id, r2, p2, now)
    assert out1[1] == out2[1] == "sequenced"
    b1, b2 = sq.acc[sq.owner == ms[0].id], sq.acc[sq.owner == ms[1].id]
    assert int((~separated_hv_matrix(b1, b2, H, V)).sum()) == 0        # Theorem 2 premise for the pair
    # both land in the same hub column, so the second waits (holds, or uses another level/time)
    d1, d2 = seq.decisions
    assert d2.hold_s + d2.pre_hold_s > 0 or d2.slot != d1.slot
    assert out1[0].meta["sequenced"] and [w.tag for w in out1[0].waypoints][-2:] == ["approach", "land"]
    rep = seq.report()
    assert rep["arrivals"] == 2 and rep["sequenced"] == 2


def test_sequencer_defers_then_goes_contingent_when_blocked(air, missions):
    sq = FcfsScheduler(air, ip=IP, rp=RP, params=SchedulerParams(terminal="sequence"))
    seq = HubSequencer(sq, RP, HubParams(max_hold_s=10.0, pre_holds_s=()))
    m = copy.deepcopy(next(x for x in missions if x.kind == "delivery"))
    r, pos = _inbound(air, m, sq)
    hub = _hub(air, m)
    blocker = np.array([[int((hub.x - 200) * 1000), int((hub.x + 200) * 1000), int((hub.y - 200) * 1000),
                         int((hub.y + 200) * 1000), 0, 200_000, 0, 10_000_000]], np.int64)   # the whole hub, all day
    sq.register(-7, blocker)
    before = sq.acc[sq.owner == m.id].copy()
    assert seq.sequence(m.id, r, pos, 1000.0, final=False) is None
    assert np.array_equal(sq.acc[sq.owner == m.id], before)           # deferred: the old plan stays registered
    v, status, _ = seq.sequence(m.id, r, pos, 1002.0, final=True)
    assert status == "contingent" and v.meta["sequenced"]


def test_arrival_priority_moves_a_ground_departure(air, missions):
    sq = FcfsScheduler(air, ip=IP, rp=RP, params=SchedulerParams(terminal="sequence"))
    seq = HubSequencer(sq, RP, HubParams(bump_after_s=0.0, max_hold_s=30.0, pre_holds_s=()))
    m = copy.deepcopy(next(x for x in missions if x.kind == "delivery"))
    r, pos = _inbound(air, m, sq)
    hub = _hub(air, m)
    column = np.array([[int((hub.x - 60) * 1000), int((hub.x + 60) * 1000), int((hub.y - 60) * 1000),
                        int((hub.y + 60) * 1000), 0, 120_000, 1_000_000, 1_600_000]], np.int64)
    sq.register(99, column)                     # a ground departure booked the hub for 10 minutes
    assert seq.sequence(m.id, r, pos, 1000.0, final=False) is None                     # not movable: no slot
    out = seq.sequence(m.id, r, pos, 1000.0, bumpable=frozenset({99}))
    assert out[1] == "sequenced" and seq.decisions[-1].bumped == (99,)                 # movable: it gives way
    assert (sq.owner == 99).sum() == 1          # the caller (the twin) withdraws and re-admits it


# ------------------------------------------------------------------ closed loop
def test_twin_with_hub_sequencing(air):
    sp = SimParams(duration_s=600, max_extra_s=900)
    vp = VehicleParams()
    req = generate_missions(air, {"delivery": 60, "emergency": 4, "inspection": 8}, 600, 3, RP)
    sched = FcfsScheduler(air, vp, IP, RP, SchedulerParams(terminal="sequence"))
    seq = HubSequencer(sched, RP)
    plant = TierAPlant(air, req, vp, sp, None, PlantParams(seed=2))
    tw = DigitalTwin(air, plant, req, sched, vp, IP, sp, lambda dt: None, TwinParams(predict_every_s=60), None, None,
                     echo=lambda *_: None, sequencer=seq)
    rep = tw.run(sp.duration_s + sp.max_extra_s)
    res = plant.sim.result()
    flown = sum(1 for d in tw.decisions if d["status"] != "rejected" and not d.get("rescheduled"))
    assert sum(s["state"] == "landed" for s in res.status) == flown
    assert rep["hubs"]["arrivals"] == flown and rep["hubs"]["sequenced"] >= flown - 1
    certified = {"accepted", "replanned", "sequenced", "awaiting"}

    def status_at(vid, t):
        return next((v["status"] for v in tw.intent_versions if v["vid"] == vid and v["valid_from"] <= t <= v["valid_to"]),
                    None)

    for e in res.los_events:          # every loss of separation involves a flight nobody certified at the time
        assert status_at(e["a"], e["t_start"]) not in certified or status_at(e["b"], e["t_start"]) not in certified
    assert rep["conformance"]["rate"] > 0.99
    assert all("cert" in v for v in tw.intent_versions)
