"""ORCA separation policy: linear program, reciprocity, separation in open air, corridor, end to end."""
import itertools
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_sim.intents import IntentParams  # noqa: E402
from swarm_sim.metrics import summarise  # noqa: E402
from swarm_sim.missions import generate_missions  # noqa: E402
from swarm_sim.orca import OrcaParams, OrcaPolicy, halfplane_max, orca_line, solve  # noqa: E402
from swarm_sim.osm import read_osm  # noqa: E402
from swarm_sim.sim import SimParams, run  # noqa: E402
from swarm_sim.vehicle import KinematicFleet, VehicleParams  # noqa: E402

OSM = ROOT / "tests/fixtures/mini_town.osm"
HUBS = [{"name": "west_depot", "x": -230, "y": -120, "pads": 6}, {"name": "east_depot", "x": 255, "y": 255, "pads": 4}]


# ------------------------------------------------------------------ linear program
def test_solve_unconstrained_returns_preferred():
    v, ok = solve([], [], 12.0, np.array([3.0, -4.0]))
    assert ok and np.allclose(v, [3.0, -4.0])


def test_solve_clamps_to_speed_circle():
    v, ok = solve([], [], 12.0, np.array([30.0, 40.0]))
    assert ok and np.isclose(np.linalg.norm(v), 12.0)


def test_solve_keeps_hard_line_when_soft_lines_conflict():
    hard = [halfplane_max(np.array([1.0, 0.0]), -2.5)]                       # v.x <= -2.5
    soft = [orca_line(np.zeros(2), np.zeros(2), np.array([-30.0, 0.0]), np.array([5.0, 0.0]), 35, 10, 0.1, 1.0)]
    v, ok = solve(hard, soft, 12.0, np.array([-3.0, 0.0]))
    assert not ok and v[0] <= -2.5 + 1e-9 and np.linalg.norm(v) <= 12.0 + 1e-9


def test_orca_line_allows_current_velocity_when_far_and_diverging():
    ln = orca_line(np.zeros(2), np.array([-5.0, 0.0]), np.array([100.0, 0.0]), np.array([5.0, 0.0]), 35, 10, 0.1)
    v = np.array([-5.0, 0.0])
    assert ln.direction[0] * (v - ln.point)[1] - ln.direction[1] * (v - ln.point)[0] >= -1e-9


@pytest.mark.parametrize("pri,pj,si,sj", list(itertools.product([0, 1, 2], [0, 1, 2], [False, True], [False, True])))
def test_shares_are_reciprocal(pri, pj, si, sj):
    pol = OrcaPolicy()
    stalled = np.array([si, sj])
    assert math.isclose(pol._share(0, 1, pri, pj, stalled) + pol._share(1, 0, pj, pri, stalled), 1.0)


# ------------------------------------------------------------------ closed loop in open air
def _fly(starts, goals, steps=900, ctx_extra=None, params=OrcaParams()):
    vp = VehicleParams()
    fleet = KinematicFleet(len(starts), vp, starts)
    fleet.on[:] = True
    pol = OrcaPolicy(params, 0.1)
    ids = np.arange(len(starts))
    dmin = np.inf
    for _ in range(steps):
        d = np.asarray(goals, float) - fleet.pos
        sp = np.linalg.norm(d[:, :2], axis=1, keepdims=True)
        desired = np.zeros((len(ids), 3))
        desired[:, :2] = d[:, :2] / np.maximum(sp, 1e-9) * np.minimum(vp.cruise_xy, sp)
        ctx = {"priority": np.ones(len(ids), int)} | (ctx_extra or {})
        fleet.command_velocity(ids, pol.adjust(fleet, ids, desired, ctx))
        fleet.step(0.1)
        for a, b in itertools.combinations(ids, 2):
            dmin = min(dmin, float(np.hypot(*(fleet.pos[a, :2] - fleet.pos[b, :2]))))
    return fleet, dmin, pol


@pytest.mark.parametrize("angle", [180, 135, 90])
def test_two_vehicles_keep_horizontal_separation(angle):
    """Same altitude, converging at `angle` degrees; without ORCA they would meet at the origin."""
    a = math.radians(angle)
    starts = [(-300.0, 0.0, 60.0), (-300.0 * math.cos(a), -300.0 * math.sin(a), 60.0)]
    goals = [(300.0, 0.0, 60.0), (300.0 * math.cos(a), 300.0 * math.sin(a), 60.0)]
    fleet, dmin, _ = _fly(starts, goals)
    assert dmin >= 30.0                                              # outside the LoS cylinder
    assert all(np.hypot(*(fleet.pos[i, :2] - np.array(goals[i][:2]))) < 5.0 for i in range(2))


def test_four_way_crossing_keeps_separation():
    R = 300.0
    starts = [(R * math.cos(t), R * math.sin(t), 60.0) for t in np.radians([0, 90, 180, 270])]
    goals = [(-x, -y, z) for x, y, z in starts]
    _, dmin, _ = _fly(starts, goals, steps=1000)
    assert dmin >= 30.0


def test_adjacent_layer_is_not_a_conflict():
    """15 m apart vertically (adjacent cruise layers): ORCA must ignore it and both fly straight."""
    starts = [(-300.0, 0.0, 55.0), (300.0, 0.0, 70.0)]
    goals = [(300.0, 0.0, 55.0), (-300.0, 0.0, 70.0)]
    fleet, _, pol = _fly(starts, goals)
    assert pol.stats["constrained"] == 0
    assert np.allclose(fleet.pos[:, 1], 0.0, atol=1e-6)


def test_corridor_holds_against_a_pushing_neighbour():
    """Vehicle 0 holds a point (vertical leg); vehicle 1 flies through it. Vehicle 0 may dodge only
    within its corridor (15 m), plus the overshoot of its dynamics."""
    p = OrcaParams()
    starts = [(0.0, 0.0, 60.0), (-300.0, 5.0, 60.0)]
    goals = [(0.0, 0.0, 60.0), (300.0, 5.0, 60.0)]
    legs = {"leg_a": np.array([[0.0, 0.0, 80.0], [-300.0, 5.0, 60.0]]),
            "leg_b": np.array([[0.0, 0.0, 60.0], [300.0, 5.0, 60.0]])}
    vp = VehicleParams()
    fleet = KinematicFleet(2, vp, starts)
    fleet.on[:] = True
    pol = OrcaPolicy(p, 0.1)
    ids = np.arange(2)
    worst = 0.0
    for _ in range(900):
        d = np.asarray(goals) - fleet.pos
        sp = np.linalg.norm(d[:, :2], axis=1, keepdims=True)
        desired = np.zeros((2, 3))
        desired[:, :2] = d[:, :2] / np.maximum(sp, 1e-9) * np.minimum(vp.cruise_xy, np.sqrt(2 * vp.a_xy_max * sp) * 0.9)
        fleet.command_velocity(ids, pol.adjust(fleet, ids, desired, legs | {"priority": np.ones(2, int)}))
        fleet.step(0.1)
        worst = max(worst, float(np.hypot(*fleet.pos[0, :2])))
    assert worst <= p.corridor_m + 5.0
    assert worst > 1.0                                             # it did dodge


# ------------------------------------------------------------------ end to end
@pytest.fixture(scope="module")
def air():
    return read_osm(OSM, hubs=HUBS)


def test_orca_end_to_end_beats_baseline(air):
    rates = {"delivery": 120, "emergency": 8, "inspection": 16}
    sp = SimParams(duration_s=600)
    vp = VehicleParams()
    missions = generate_missions(air, rates, 600, seed=4)
    base = summarise(run(air, missions, vp, sp), vp, IntentParams())
    pol = OrcaPolicy(OrcaParams(), sp.dt)
    res = run(air, generate_missions(air, rates, 600, seed=4), vp, sp, pol)
    met = summarise(res, vp, IntentParams())
    assert met["completed"] == met["missions"] == base["missions"]
    assert met["intrusions"] == {"building": 0, "no_fly": 0}
    assert met["conformance"]["rate"] >= 0.99
    assert met["los_events"]["en_route"] <= base["los_events"]["en_route"]
    assert res.params["policy"] == "orca" and res.params["policy_info"]["stats"]["agent_steps"] > 0
