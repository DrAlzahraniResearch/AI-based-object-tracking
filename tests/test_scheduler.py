"""Intent phases and slicing, and the FCFS strategic scheduler."""
import copy
import itertools
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_sim.intents import Box4, IntentParams, mission_intents, separated_hv_matrix, separated_hv_ref  # noqa: E402
from swarm_sim.missions import Mission, RoutingParams, Waypoint, generate_missions  # noqa: E402
from swarm_sim.osm import read_osm  # noqa: E402
from swarm_sim.scheduler import FcfsScheduler, SchedulerParams, variant  # noqa: E402
from swarm_sim.vehicle import VehicleParams  # noqa: E402

OSM = ROOT / "tests/fixtures/mini_town.osm"
HUBS = [{"name": "west_depot", "x": -230, "y": -120, "pads": 6}, {"name": "east_depot", "x": 255, "y": 255, "pads": 4}]
H, V = 30_000, 10_000


@pytest.fixture(scope="module")
def air():
    return read_osm(OSM, hubs=HUBS)


@pytest.fixture(scope="module")
def missions(air):
    return generate_missions(air, {"delivery": 60, "emergency": 4, "inspection": 8}, 900, seed=3)


def _delivery(mid=0, t=0.0, x=150.0, y=0.0, z=40.0, back=55.0):
    wps = [Waypoint(0, 0, z, tag="climb"), Waypoint(x, y, z, tag="cruise"), Waypoint(x, y, 5, 20, "drop"),
           Waypoint(x, y, back, tag="climb"), Waypoint(0, 0, back, tag="cruise"), Waypoint(0, 0, 0, tag="land")]
    return Mission(mid, "delivery", 1, t, "hub", 0, wps, {"start": [0.0, 0.0, 0.0]})


# ------------------------------------------------------------------ intents
def test_phases_and_vertical_slicing():
    m = _delivery()
    ip = IntentParams()
    boxes = mission_intents(m, VehicleParams(), ip)
    term = [b for b in boxes if b.phase == "terminal"]
    assert term and all(max(abs(b.x0 + ip.lateral_buffer_m), abs(b.x1 - ip.lateral_buffer_m)) <= ip.terminal_radius_m
                        for b in term)
    drop_leg = [b for b in boxes if b.leg == 2]                  # cruise (40 m) -> drop (5 m)
    assert len(drop_leg) >= 4                                    # 35 m in <= 10 m slices
    assert all(b.z1 - b.z0 <= ip.max_piece_z_m + 2 * ip.vertical_buffer_m + 1e-9 for b in drop_leg)
    assert all(a.t1 <= b.t1 for a, b in zip(drop_leg, drop_leg[1:]))   # slices are sequential in time


def test_adjacent_layers_are_certifiable():
    """Boxes on layers 15 m apart are exactly v = 10 m apart: Theorem 2 certifies them, Theorem 1 cannot."""
    ip = IntentParams()
    a = Box4(0, 0, 0, 100, 0, 10, 40 - ip.vertical_buffer_m, 40 + ip.vertical_buffer_m, 0, 100).as_int()
    b = Box4(1, 0, 0, 100, 0, 10, 55 - ip.vertical_buffer_m, 55 + ip.vertical_buffer_m, 0, 100).as_int()
    assert separated_hv_ref(a, b, H, V)
    assert not separated_hv_ref(a, b, H, V + 1)


# ------------------------------------------------------------------ variants
def test_variant_delay_and_layer_bump(air):
    m = _delivery(z=40.0, back=55.0)
    v = variant(m, air, RoutingParams(), 30.0, 1, 2)
    assert v.release_t == 30.0 and m.release_t == 0.0                    # original untouched
    assert [w.z for w in v.waypoints] == [70.0, 70.0, 5, 115.0, 115.0, 0]
    assert variant(m, air, RoutingParams(), 0.0, 0, 3) is None            # 55 + 90 > 120 m ceiling


# ------------------------------------------------------------------ scheduler
def test_identical_requests_get_separated(air):
    a, b = _delivery(0, 0.0), _delivery(1, 0.0)
    flown, rep = FcfsScheduler(air).schedule([a, b])
    assert rep["accepted"] == 2
    second = [d for d in rep["decisions"] if d["mission"] == 1][0]
    assert second["ground_delay_s"] > 0 or second["bump_out"] or second["bump_back"]


def _en_route(m):
    return np.array([b.as_int() for b in mission_intents(m, VehicleParams(), IntentParams())
                     if b.phase == "en_route"], np.int64).reshape(-1, 8)


def test_accepted_plans_are_pairwise_certified(air, missions):
    flown, rep = FcfsScheduler(air).schedule(copy.deepcopy(missions))
    ok = [f for f in flown if f.meta["schedule_status"] == "accepted"]
    assert len(ok) >= 0.8 * len(missions)
    for f, g in itertools.combinations(ok, 2):
        A, B = _en_route(f), _en_route(g)
        if len(A) and len(B):
            assert separated_hv_matrix(A, B, H, V).all(), (f.id, g.id)


def test_emergencies_never_delayed_and_ids_are_positions(air, missions):
    flown, rep = FcfsScheduler(air).schedule(copy.deepcopy(missions))
    assert [f.id for f in flown] == list(range(len(flown)))
    assert all(f.release_t >= f.meta["requested_t"] for f in flown)
    for d in rep["decisions"]:
        if d["kind"] == "emergency":
            assert d["status"] != "rejected" and d["ground_delay_s"] == 0.0
        if d["status"] == "rejected":
            assert d["kind"] != "emergency"
    assert rep["accepted"] + rep["accepted_uncertified"] + rep["rejected"] == len(missions)


def test_scheduler_is_deterministic(air, missions):
    _, r1 = FcfsScheduler(air).schedule(copy.deepcopy(missions))
    _, r2 = FcfsScheduler(air).schedule(copy.deepcopy(missions))
    assert r1["decisions"] == r2["decisions"]


def test_tight_delay_budget_rejects(air):
    reqs = [_delivery(i, 0.0) for i in range(4)]
    flown, rep = FcfsScheduler(air, params=SchedulerParams(max_delay_s=0.0, max_layer_bump=0)).schedule(reqs)
    assert rep["accepted"] == 1 and rep["rejected"] == 3 and len(flown) == 1


def test_verified_search_matches_reference(air, missions):
    try:
        from runtime.utm_checker import CheckerUnavailable, VerifiedChecker
        chk = VerifiedChecker()
    except Exception as e:  # CheckerUnavailable, or the library missing in the eval image
        pytest.skip(f"verified checker not available: {e}")
    sub = copy.deepcopy(missions[:25])
    _, r_ref = FcfsScheduler(air).schedule(copy.deepcopy(sub))
    _, r_ver = FcfsScheduler(air, checker=chk).schedule(copy.deepcopy(sub))
    assert r_ref["decisions"] == r_ver["decisions"]
