"""OSM conversion, Tier A dynamics, missions, intents and the reference checker."""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from shapely.geometry import LineString, Point, Polygon

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_sim.intents import IntentParams, inside_ref, mission_intents, separated_ref  # noqa: E402
from swarm_sim.metrics import summarise  # noqa: E402
from swarm_sim.missions import RoutingParams, cruise_layer, generate_missions, route_clear  # noqa: E402
from swarm_sim.osm import LocalFrame, building_height, hub_clearance_problems, parse_length_m, read_osm, world_sdf  # noqa: E402
from swarm_sim.sim import SimParams, run  # noqa: E402
from swarm_sim.vehicle import KinematicFleet, VehicleParams  # noqa: E402

OSM = ROOT / "tests/fixtures/mini_town.osm"
HUBS = [{"name": "west_depot", "x": -230, "y": -120, "pads": 6}, {"name": "east_depot", "x": 255, "y": 255, "pads": 4}]


@pytest.fixture(scope="module")
def air():
    return read_osm(OSM, hubs=HUBS)


# ------------------------------------------------------------------ OSM
def test_local_frame_round_trip_and_scale():
    f = LocalFrame(40.0, -105.0)
    x, y = f.to_xy(40.001, -104.999)
    assert abs(float(y) - 111.195) < 0.05                       # 0.001 deg latitude ~ 111.2 m
    assert abs(float(x) - 111.195 * math.cos(math.radians(40))) < 0.05
    lat, lon = f.to_latlon(x, y)
    assert abs(float(lat) - 40.001) < 1e-9 and abs(float(lon) + 104.999) < 1e-9


@pytest.mark.parametrize("s,v", [("12", 12.0), ("12 m", 12.0), ("12.5m", 12.5), ("40'", 12.192), ("40 ft", 12.192),
                                 ("", None), ("tall", None)])
def test_parse_length(s, v):
    out = parse_length_m(s)
    assert (out is None and v is None) or abs(out - v) < 1e-6


def test_building_height_precedence():
    assert building_height({"building": "yes", "height": "20", "building:levels": "2"}) == (20.0, 0.0, "tag")
    assert building_height({"building": "yes", "building:levels": "4"}) == (12.0, 0.0, "levels")
    assert building_height({"building": "house"})[0] == 6.0
    top, base, _ = building_height({"building": "yes", "min_height": "5", "height": "15"})
    assert (top, base) == (15.0, 5.0)


def test_read_fixture(air):
    assert len(air.buildings) == 65                     # 64 ways + 1 multipolygon relation
    srcs = {b.height_source for b in air.buildings}
    assert srcs == {"tag", "levels", "default"}
    assert len(air.no_fly) == 1 and air.no_fly[0].reason == "landuse=military"
    assert len(air.roads) == 10 and [h.name for h in air.hubs] == ["west_depot", "east_depot"]
    assert max(b.height for b in air.buildings) == 45.0
    school = next(b for b in air.buildings if b.osm_id == 9001)
    assert school.kind == "school" and len(school.polygon) == 7      # L-shape, closed ring
    assert hub_clearance_problems(air) == []


def test_queries(air):
    b = max(air.buildings, key=lambda b: b.height)
    c = np.mean(b.polygon[:-1], axis=0)
    assert air.inside_building(c[0], c[1], b.height - 1) and not air.inside_building(c[0], c[1], b.height + 1)
    nf = np.mean(air.no_fly[0].polygon[:-1], axis=0)
    assert air.in_no_fly(nf[0], nf[1], 50) and not air.in_no_fly(-300, 200, 50)
    assert air.building_top_along((c[0] - 100, c[1]), (c[0] + 100, c[1])) >= b.height


@pytest.mark.parametrize("geom", ["polyline", "box"])
def test_world_sdf_is_valid_xml(air, geom):
    root = ET.fromstring(world_sdf(air, geom))
    models = root.find("world").findall("model")
    names = [m.get("name") for m in models]
    assert sum(n.startswith("bldg_") for n in names) == 65
    assert {"ground", "roads", "hub_west_depot", "hub_east_depot"} <= set(names)
    assert root.find("world/spherical_coordinates/latitude_deg") is not None
    bld = next(m for m in models if m.get("name").startswith("bldg_"))
    assert bld.find(f"link/visual/geometry/{geom}") is not None and bld.find("link/collision") is not None


# ------------------------------------------------------------------ vehicle
def test_vehicle_respects_limits():
    vp = VehicleParams()
    f = KinematicFleet(2, vp, [[0, 0, 0], [0, 0, 50]])
    f.on[:] = True
    f.command_velocity([0, 1], [[100, 0, 100], [0, 0, -100]])
    speeds, accs, prev = [], [], f.vel.copy()
    for _ in range(450):                                         # 45 s: 50 m at <= 1.5 m/s takes > 33 s
        f.step(0.1)
        accs.append(np.linalg.norm((f.vel - prev)[:, :2], axis=1) / 0.1)
        prev = f.vel.copy()
        speeds.append(f.vel.copy())
    s = np.array(speeds)
    assert np.linalg.norm(s[:, 0, :2], axis=1).max() <= vp.v_xy_max + 1e-9
    assert s[:, 0, 2].max() <= vp.v_up_max + 1e-9 and s[:, 1, 2].min() >= -vp.v_down_max - 1e-9
    assert np.max(accs) <= vp.a_xy_max + 1e-9
    assert f.pos[1, 2] == 0.0                                    # stopped by the ground


# ------------------------------------------------------------------ missions + routing
def test_semicircular_layers(air):
    rp = RoutingParams()
    z_e = cruise_layer(air, (-300, 250), (-200, 250), rp)        # eastbound, open area
    z_w = cruise_layer(air, (-200, 250), (-300, 250), rp)        # westbound
    assert z_e != z_w and abs(z_e - z_w) == rp.layer_step_m


def test_missions_clear_buildings_and_no_fly(air):
    rp = RoutingParams()
    ms = generate_missions(air, {"delivery": 60, "emergency": 6, "inspection": 20}, 1800, seed=7, rp=rp)
    assert len(ms) > 30 and {m.kind for m in ms} == {"delivery", "emergency", "inspection"}
    for m in ms:
        assert route_clear(air, tuple(m.meta["start"]), m.waypoints, rp)
        assert m.waypoints[-1].z == m.meta["start"][2]
        prev = m.meta["start"][:2]
        for w in m.waypoints:                         # every leg keeps the no-fly margin (ORCA corridor + 5 m)
            seg = LineString([prev, (w.x, w.y)]) if tuple(prev) != (w.x, w.y) else Point(w.x, w.y)
            assert all(Polygon(n.polygon).distance(seg) > rp.no_fly_margin_m for n in air.no_fly)
            prev = (w.x, w.y)
    assert generate_missions(air, {"delivery": 60}, 1800, 7, rp)[0].to_json() == \
        generate_missions(air, {"delivery": 60}, 1800, 7, rp)[0].to_json()      # deterministic


def test_sim_end_to_end(air):
    vp, ip = VehicleParams(), IntentParams()
    ms = generate_missions(air, {"delivery": 40, "emergency": 4, "inspection": 8}, 900, seed=3)
    res = run(air, ms, vp, SimParams(duration_s=900))
    assert all(s["state"] == "landed" for s in res.status)
    assert res.intrusions == {"building": [], "no_fly": []}
    met = summarise(res, vp, ip)
    assert met["conformance"]["rate"] == 1.0                     # flights stay inside their own intents
    assert abs(met["delay_s_vs_plan"]["median"]) < 30            # plan timing is close to flown timing
    for e in res.los_events:
        assert e["min_dist_m"] <= math.hypot(SimParams().h_sep_m, SimParams().v_sep_m)


# ------------------------------------------------------------------ intents + reference checker
def test_reference_checker_matches_definition():
    a = (0, 10, 0, 10, 0, 10, 0, 10)
    assert separated_ref(a, (15, 20, 0, 10, 0, 10, 0, 10), 5)
    assert not separated_ref(a, (14, 20, 0, 10, 0, 10, 0, 10), 5)
    assert separated_ref(a, (0, 10, 0, 10, 0, 10, 11, 20), 5)
    assert inside_ref(a, 2, 5, 5, 5, 5) and not inside_ref(a, 2, 9, 5, 5, 5)


def test_reference_checker_is_sound_on_random_boxes():
    """Property from Theorem 1: checker says separated -> every sampled pair of co-temporal points is >= buf apart."""
    rng = np.random.default_rng(0)
    for _ in range(3000):
        lo = rng.integers(-1000, 1000, (2, 4))
        hi = lo + rng.integers(0, 600, (2, 4))
        A = tuple(int(v) for pair in zip(lo[0], hi[0]) for v in pair)
        B = tuple(int(v) for pair in zip(lo[1], hi[1]) for v in pair)
        buf = int(rng.integers(0, 300))
        if not separated_ref(A, B, buf):
            continue
        for _ in range(20):
            t = rng.integers(max(A[6], B[6]), min(A[7], B[7]) + 1) if max(A[6], B[6]) <= min(A[7], B[7]) else None
            if t is None:
                break
            p = [rng.uniform(A[2 * k], A[2 * k + 1]) for k in range(3)]
            q = [rng.uniform(B[2 * k], B[2 * k + 1]) for k in range(3)]
            assert math.dist(p, q) >= buf - 1e-9


def test_intents_cover_route_and_round_outwards(air):
    ms = generate_missions(air, {"delivery": 20}, 600, seed=1)
    m = ms[0]
    boxes = mission_intents(m, VehicleParams(), IntentParams())
    for w in m.waypoints:
        assert any(b.contains(w.x, w.y, w.z, b.t0) or (b.x0 <= w.x <= b.x1 and b.y0 <= w.y <= b.y1 and b.z0 <= w.z <= b.z1)
                   for b in boxes)
    b = boxes[0]
    i = b.as_int()
    assert i[0] <= b.x0 * 1000 and i[1] >= b.x1 * 1000 and i[6] <= b.t0 * 1000 and i[7] >= b.t1 * 1000
