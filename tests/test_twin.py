"""Digital twin (milestone 4.5): state estimation, conformance, calibration, incremental scheduling,
simulation hooks, logging, and a short closed-loop run against the Tier A plant.

The PX4 plant test runs only with RUN_PX4=1 and a PX4 SITL build (PX4_ROOT); the verified audit test only
with the checker library built."""
import copy
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_sim.intents import (Box4, IntentParams, inflate_ref, separated_dev_ref,  # noqa: E402
                               separated_hv_matrix)
from swarm_sim.missions import Mission, RoutingParams, Waypoint, generate_missions  # noqa: E402
from swarm_sim.osm import Airspace  # noqa: E402
from swarm_sim.scheduler import FcfsScheduler, SchedulerParams  # noqa: E402
from swarm_sim.sim import SimParams, Simulation  # noqa: E402
from swarm_sim.twin.core import DigitalTwin, TwinParams, score_los_predictions  # noqa: E402
from swarm_sim.twin.log import TwinLog, read_log  # noqa: E402
from swarm_sim.twin.state import ConformanceMonitor, SpeedCalibrator, TwinState  # noqa: E402
from swarm_sim.twin.telemetry import PlantParams, ReplayPlant, Telemetry, TierAPlant  # noqa: E402
from swarm_sim.vehicle import KinematicFleet, VehicleParams  # noqa: E402

WORLD = ROOT / "data/worlds/mini_town/airspace.json"


@pytest.fixture(scope="module")
def air():
    if not WORLD.exists():
        pytest.skip("mini_town world not built (scripts/osm_to_world.py)")
    return Airspace.load(WORLD)


def _z(vid, t, pos, vel=(0, 0, 0), state="flying", wp=1, hold=-1.0, ver=1):
    return Telemetry(vid, t, np.array(pos, float), np.array(vel, float), state, wp, hold, ver)


# ------------------------------------------------------------------ state estimation
def test_state_filter_tracks_and_compensates_latency():
    st = TwinState()
    rng = np.random.default_rng(0)
    for k in range(50):
        t = k * 0.2
        st.ingest(_z(1, t, [10 * t + rng.normal(0, 0.7), rng.normal(0, 0.7), 50], [10, 0, 0]))
    est = st.estimate(1, 9.8 + 0.25)                            # 0.25 s link latency
    assert abs(est[0] - 10 * 10.05) < 1.5 and abs(est[1]) < 1.5


def test_state_ignores_progress_of_a_stale_route():
    st = TwinState()
    st.ingest(_z(3, 0.0, [0, 0, 50], wp=4))
    st.rerouted(3)                                              # twin replanned: version 2, waypoint 0
    st.ingest(_z(3, 0.2, [1, 0, 50], wp=5, ver=1))              # late sample from the old route
    assert st.tracks[3].wp_idx == 0 and st.tracks[3].route_ver == 2
    assert abs(st.tracks[3].pos[0] - 0.5) < 1e-9                # position is still used
    st.ingest(_z(3, 0.4, [2, 0, 50], wp=1, ver=2))
    assert st.tracks[3].wp_idx == 1


# ------------------------------------------------------------------ conformance
def test_conformance_monitor_confirms_and_recovers():
    mon = ConformanceMonitor(confirm_s=1.0, clear_s=1.0)
    mon.set_intents(0, [Box4(0, 0, 0, 100, -10, 10, 40, 60, 0, 100, "en_route")])
    assert mon.check(0, np.array([50, 0, 50]), 1.0) is None
    assert mon.check(0, np.array([150, 0, 50]), 2.0) is None    # one miss: not yet
    a = mon.check(0, np.array([150, 0, 50]), 3.1)
    assert a["event"] == "nonconforming" and a["phase"] == "en_route"
    assert mon.check(0, np.array([50, 0, 50]), 4.0) is None
    assert mon.check(0, np.array([50, 0, 50]), 5.1)["event"] == "conforming"
    a = None
    for t in (101.0, 102.5):                                    # past the time window: "late"
        a = mon.check(0, np.array([50, 0, 50]), t) or a
    assert a["event"] == "nonconforming" and a["phase"] == "late"
    s = mon.summary()
    assert s["samples"] == 7 and 0 < s["rate"] < 1


# ------------------------------------------------------------------ calibration
def test_speed_calibrator_finds_slower_fleet_and_degraded_vehicle():
    from swarm_sim.twin.state import Track
    cal = SpeedCalibrator(VehicleParams(), min_samples=10)
    a, b = (0, 0, 70), (400, 0, 70)

    def feed(vid, v, n):
        for k in range(n):
            cal.observe(Track(k, np.array([200.0, 0, 70]), np.array([v, 0, 0]), "flying", 1, -1), a, b, vid=vid)

    feed(9, 4.0, 40)                                             # one degraded vehicle alone: no fleet update
    assert cal.maybe_update(50.0) is None and abs(cal.vehicle_speed(9) - 4.0) < 1e-9
    for vid in range(3):
        feed(vid, 8.5, 15)
    vp = cal.maybe_update(100.0)                                 # 3 of 4 vehicles at 8.5: fleet 8.5
    assert vp is not None and abs(vp.cruise_xy - 8.5) < 0.1 and vp.v_xy_max == vp.cruise_xy
    assert cal.vehicle_speed(42) is None
    assert cal.maybe_update(130.0) is None                       # no drift left


# ------------------------------------------------------------------ Theorem 3 (Python reference)
def test_inflate_and_dev_reference_match_theorem3_form():
    a, b = (0, 10, 0, 10, 0, 10, 0, 10), (15, 20, 0, 10, 0, 10, 0, 10)
    assert inflate_ref(a, 2, 1, 3) == (-2, 12, -2, 12, -1, 11, -3, 13)
    assert separated_dev_ref(a, b, 1, 100, 2, 0, 0) and not separated_dev_ref(a, b, 1, 100, 3, 0, 0)
    # sepHV_inflate_iff: widened boxes separated  <=>  nominal gaps h + 2 eh / v + 2 ev / time 2 dt
    rng = np.random.default_rng(3)
    for _ in range(2000):
        x = tuple(int(v) for v in np.sort(rng.integers(-500, 500, (4, 2)), axis=1).ravel())
        y = tuple(int(v) for v in np.sort(rng.integers(-500, 500, (4, 2)), axis=1).ravel())
        h, v, eh, ev, dt = (int(q) for q in rng.integers(0, 100, 5))
        gaps = (x[7] + 2 * dt < y[6] or y[7] + 2 * dt < x[6] or x[1] + h + 2 * eh <= y[0] or y[1] + h + 2 * eh <= x[0]
                or x[3] + h + 2 * eh <= y[2] or y[3] + h + 2 * eh <= x[2] or x[5] + v + 2 * ev <= y[4]
                or y[5] + v + 2 * ev <= x[4])
        assert separated_dev_ref(x, y, h, v, eh, ev, dt) == gaps


# ------------------------------------------------------------------ simulation hooks
def test_fleet_speed_cap_is_identity_when_unset_and_limits_when_set():
    f = KinematicFleet(2, VehicleParams())
    f.on[:] = True
    f.command_velocity(np.array([0, 1]), np.array([[20.0, 0, 0], [20.0, 0, 0]]))
    assert np.allclose(f.cmd[:, 0], 12.0)
    f.v_cap[1] = 4.0
    f.command_velocity(np.array([0, 1]), np.array([[20.0, 0, 0], [20.0, 0, 0]]))
    assert np.allclose(f.cmd[:, 0], [12.0, 4.0])


def _delivery(x0, y0, x1, y1, t=0.0, mid=0):
    wps = [Waypoint(x0, y0, 55, tag="climb"), Waypoint(x1, y1, 55, tag="cruise"),
           Waypoint(x1, y1, 5, 5, "drop"), Waypoint(x1, y1, 70, tag="climb"),
           Waypoint(x0, y0, 70, tag="cruise"), Waypoint(x0, y0, 0, tag="land")]
    return Mission(mid, "delivery", 1, t, "h", 0, wps, {"start": [x0, y0, 0.0]})


def test_simulation_dispatch_set_route_cancel_and_fork(air):
    m0, m1 = _delivery(0, 0, 300, 0), _delivery(0, 50, -300, 50, mid=1)
    a, b = copy.deepcopy(m0), copy.deepcopy(m1)
    a.release_t = b.release_t = math.inf
    sim = Simulation(air, [a, b], VehicleParams(), SimParams(duration_s=0, max_extra_s=600), None, monitors=False)
    m0.release_t = 1.0
    sim.dispatch(0, m0)
    sim.cancel(1)
    for _ in range(300):
        sim.step()
    assert sim.state[0] == "flying" and sim.state[1] == "cancelled"
    p = sim.fleet.pos[0].copy()
    r = copy.deepcopy(m0)
    r.waypoints = [Waypoint(p[0], p[1], 55, 0, "cruise"), Waypoint(p[0], p[1], 0, tag="land")]
    r.meta = dict(r.meta) | {"start": p.tolist()}
    sim.set_route(0, r)                                         # replan: land where it is
    fork = Simulation.from_state(air, [r], VehicleParams(), sim.sp, None, sim.fleet.t, sim.fleet.pos[:1],
                                 sim.fleet.vel[:1], np.array(["flying"], dtype=object), np.zeros(1, int), np.full(1, -1.0))
    while sim.step():
        pass
    while fork.step():
        pass
    assert sim.state[0] == "landed" and fork.state[0] == "landed"
    assert np.hypot(*(sim.fleet.pos[0, :2] - p[:2])) < 15
    assert np.allclose(sim.fleet.pos[0], fork.fleet.pos[0], atol=1e-9)    # same model, same state: same future


# ------------------------------------------------------------------ incremental scheduler
def test_incremental_admit_withdraw_replan(air):
    sch = FcfsScheduler(air, params=SchedulerParams(max_delay_s=300))
    a = _delivery(-300, 0, 300, 0, 0.0, 0)
    b = _delivery(300, 20, -300, 20, 0.0, 1)                    # head-on on the same layer
    b.waypoints[0].z = b.waypoints[1].z = 55
    va, da, ba = sch.admit(a)
    vb, db, bb = sch.admit(b)
    assert da.status == "accepted" and db.status == "accepted" and (db.ground_delay_s > 0 or db.bump_out > 0)
    assert int((~separated_hv_matrix(ba, bb, sch.h_mm, sch.v_mm)).sum()) == 0
    n = len(sch.acc)
    sch.withdraw(1)
    assert len(sch.acc) == n - len(bb) and set(sch.owner.tolist()) == {0}
    sch.register(1, bb)
    # replan flight 0 from mid-air: certified against flight 1's boxes, old boxes replaced
    m = copy.deepcopy(va)
    m.waypoints = m.waypoints[1:]
    m.meta = dict(m.meta) | {"start": [0.0, 0.0, 55.0], "pad": [-300, 0, 0]}
    v, status, hover, boxes = sch.replan(0, m, now=40.0, max_hover_s=120.0)
    assert status in ("replanned", "contingent")
    if status == "replanned":
        assert sch.conflicts_of(0, boxes) == 0
    assert (sch.owner == 0).sum() == len(boxes)
    sch.prune(1e6)
    assert len(sch.acc) == 0


# ------------------------------------------------------------------ log
def test_twin_log_roundtrip(tmp_path):
    log = TwinLog(tmp_path / "x")
    log.write("/twin/alert", 1.5, {"vid": 3, "event": "nonconforming"})
    log.write("/twin/state", 2.0, {"vehicles": {}})
    log.close()
    rows = list(read_log(log.file))
    assert [r[0] for r in rows] == ["/twin/alert", "/twin/state"] and rows[0][1] == 1.5 and rows[0][2]["vid"] == 3
    assert [r[0] for r in read_log(log.file, ["/twin/state"])] == ["/twin/state"]


def test_score_los_predictions():
    preds = [{"t": 100.0, "los": [{"a": 2, "b": 1, "t_onset": 130.0}]},                    # pair order is irrelevant
             {"t": 110.0, "los": [{"a": 1, "b": 2, "t_onset": 128.0}, {"a": 3, "b": 4, "t_onset": 150.0}]}]
    events = [{"a": 1, "b": 2, "t_start": 129.0, "terminal": False}, {"a": 5, "b": 6, "t_start": 200.0, "terminal": False},
              {"a": 7, "b": 8, "t_start": 50.0, "terminal": True}]
    s = score_los_predictions(preds, events)
    assert s["actual_en_route"] == 2 and s["recall_any_lead"] == 0.5 and s["lead_s_median"] == 29.0
    assert s["predicted_events"] == 2 and s["precision"] == 0.5 and s["recall_lead_30s"] == 0.0


# ------------------------------------------------------------------ closed loop
@pytest.fixture(scope="module")
def short_twin(air, tmp_path_factory):
    rp, ip, vp = RoutingParams(), IntentParams(), VehicleParams()
    sp = SimParams(duration_s=600, max_extra_s=900)
    req = generate_missions(air, {"delivery": 60, "emergency": 4, "inspection": 8}, 600, 4, rp)
    pp = PlantParams(vehicle={"v_xy_max": 8.5}, faults=({"t": 200, "nth_flying": 0, "v_xy_max": 4.0},), seed=1)
    plant = TierAPlant(air, req, vp, sp, None, pp)
    out = tmp_path_factory.mktemp("twin")
    log = TwinLog(out / "twin_log")
    tw = DigitalTwin(air, plant, req, FcfsScheduler(air, vp, ip, rp), vp, ip, sp, lambda dt: None,
                     TwinParams(predict_every_s=20), None, log, echo=lambda *_: None)
    rep = tw.run(sp.duration_s + sp.max_extra_s)
    log.close()
    return tw, rep, plant, log.file, req


def test_closed_loop_twin(short_twin):
    tw, rep, plant, _, req = short_twin
    res = plant.sim.result()
    flown = sum(d["status"] != "rejected" for d in tw.decisions)
    assert len(tw.decisions) == len(req) and sum(s["state"] == "landed" for s in res.status) == flown
    assert rep["conformance"]["rate"] > 0.98
    up = [u for u in rep["calibration"]["updates"] if "cruise_xy" in u]
    assert up and abs(up[0]["cruise_xy"] - 8.5) < 0.2                      # learned the plant's true limit
    faulty = plant.fault_log[0]["vid"]
    assert any(r["vid"] == faulty and r["vehicle_cruise_xy"] and r["vehicle_cruise_xy"] < 5 for r in tw.replans)
    assert rep["prediction"]["position_error_m"]["60s"]["median"] < 3.0
    for v in tw.intent_versions:                                             # every version closed, every vid known
        assert v["valid_to"] is not None and v["valid_from"] <= v["valid_to"]


def test_replay_shadow_mode_reproduces_telemetry(short_twin, air):
    tw, rep, _, logfile, req = short_twin
    frames, n = {}, 0
    for _t, _tt, msg in read_log(logfile, ["/twin/telemetry"]):
        for s in msg["samples"]:
            n += 1
            frames.setdefault(s["t"], []).append(Telemetry(s["vid"], s["t"], np.array(s["pos"]), np.array(s["vel"]),
                                                           s["state"], s["wp_idx"], s["hold_until"], s["route_ver"]))
    plant = ReplayPlant(sorted(frames.items()), 0.1)
    got = 0
    while plant.advance():
        got += len(plant.poll())
    got += len(plant.poll())
    assert got == n and n > 1000


def test_verified_audit_of_twin_run(short_twin, tmp_path):
    try:
        from runtime.utm_checker import CheckerUnavailable, VerifiedChecker
        VerifiedChecker()
    except (ImportError, CheckerUnavailable, OSError) as e:
        pytest.skip(f"verified checker not built: {e}")
    tw, rep, plant, _, _ = short_twin
    res = plant.sim.result()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    np.savez_compressed(run_dir / "truth.npz", times=res.times, tracks=res.tracks)
    (run_dir / "events.json").write_text(json.dumps({"los": res.los_events}))
    (run_dir / "intents.json").write_text(json.dumps({"versions": tw.intent_versions}))
    r = subprocess.run([sys.executable, str(ROOT / "scripts/audit_twin.py"), str(run_dir)], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    a = json.loads((run_dir / "audit.json").read_text())
    assert a["claims"]["violations"] == 0 and a["los"]["counterexample_samples"] == 0
    assert a["claims"]["concurrent_pairs_with_certified_claim"] > 10


@pytest.mark.skipif(os.environ.get("RUN_PX4") != "1", reason="set RUN_PX4=1 (needs a PX4 SITL build, PX4_ROOT)")
def test_px4_plant_flies_a_short_mission(air):  # pragma: no cover - integration
    from swarm_sim.twin.px4 import Px4Plant
    m = _delivery(0, 0, 60, 0, 2.0)
    m.waypoints[2] = Waypoint(60, 0, 20, 2, "drop")
    plant = Px4Plant(1, VehicleParams(), SimParams(), speed_factor=2)
    plant.t_end = 0.0                                           # advance() is False once everything has landed
    try:
        plant.dispatch(0, m)
        seen = []
        while plant.advance() and plant.now() < 400:
            seen += plant.poll()
        seen += plant.poll()
        assert any(z.state == "landed" for z in seen)
        assert max(z.pos[0] for z in seen) > 50
        t0 = plant.now()
        for _ in range(20):
            plant.advance()
        assert plant.now() > t0                                 # the clock keeps running after landing
    finally:
        plant.close()
