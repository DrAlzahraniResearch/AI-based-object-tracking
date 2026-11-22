"""Visualization and Gazebo integration (milestone 6): scene frames, the Gazebo viewer's calls, the live link,
Foxglove export, PX4-in-Gazebo helpers and worlds.

Everything here runs without Gazebo. With RUN_GZ=1 and `gz` + its Python bindings installed (Docker: service gz),
one test starts a real world and drives it; with RUN_PX4GZ=1 (service px4gz) one short PX4-in-Gazebo twin run."""
import json
import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_sim.gz.live import GzLiveTap, VersionStream, decode, encode  # noqa: E402
from swarm_sim.gz.models import (camera_rig_sdf, drone_marker_sdf, follow_pose, look_at,  # noqa: E402
                                 overview_pose, rpy_to_quat)
from swarm_sim.gz.record import ffmpeg_cmd, image_rgb  # noqa: E402
from swarm_sim.gz.transport import parse_clock_text  # noqa: E402
from swarm_sim.gz.view import PARK_Z, GzView, RecordingClient, ViewOptions, frame_markers  # noqa: E402
from swarm_sim.osm import PX4_GZ_SYSTEMS, Airspace, Building, Hub, NoFly, world_sdf  # noqa: E402
from swarm_sim.twin.log import TwinLog  # noqa: E402
from swarm_sim.twin.px4_gz import instance_env, pad_free  # noqa: E402
from swarm_sim.viz.scene import INTENT_TOPIC, Scene, world_mesh  # noqa: E402


# ------------------------------------------------------------------ fixtures: a tiny world and a synthetic run
@pytest.fixture(scope="module")
def tiny_air():
    sq = [[0, 0], [20, 0], [20, 20], [0, 20], [0, 0]]
    return Airspace(name="tiny", origin={"lat": 52.0, "lon": 4.0}, bounds=[-100, -100, 300, 300],
                    buildings=[Building(1, [[x + 50, y + 50] for x, y in sq], 15.0)],
                    no_fly=[NoFly(2, [[200, 200], [250, 200], [250, 250], [200, 250], [200, 200]], "test", 0, 60)],
                    hubs=[Hub("H", 0.0, 0.0, pads=4, pad_spacing_m=8.0)])


def _version(vid, t0, t1, boxes, cert, status="accepted"):
    return {"vid": vid, "valid_from": t0, "valid_to": t1, "status": status, "int": boxes, "cert": cert,
            "phase": ["en_route"] * len(boxes), "flow": ["dep"] * len(boxes)}


def _box(x0, x1, y0, y1, z0, z1, t0, t1):
    return [int(v * 1000) for v in (x0, x1, y0, y1, z0, z1, t0, t1)]


@pytest.fixture()
def run_dir(tmp_path, tiny_air):
    wd = tmp_path / "worlds" / "tiny"
    wd.mkdir(parents=True)
    tiny_air.save(wd / "airspace.json")
    run = tmp_path / "run"
    run.mkdir()
    (run / "report.json").write_text(json.dumps({"world": "tiny", "params": {"twin": {"log_state_hz": 1.0}}}))
    versions = [_version(0, 0.0, 15.0, [_box(0, 10, 0, 10, 0, 40, 0, 12), _box(0, 100, 0, 10, 35, 45, 8, 30)], [True, True]),
                _version(0, 15.0, None, [_box(10, 110, 0, 10, 35, 45, 15, 40)], [False], "contingent"),
                _version(1, 0.0, None, [_box(-5, 5, -5, 5, 0, 30, 0, 50)], [True])]
    (run / "intents.json").write_text(json.dumps({"units": {"int": "mm, ms"}, "versions": versions}))
    log = TwinLog(run / "twin_log")
    for t in range(0, 21):
        veh = {"0": {"pos": [float(t), 0.0, 40.0], "conforming": t < 10, "wp_idx": 1}}
        if t <= 5:
            veh["1"] = {"pos": [0.0, 0.0, float(t)], "conforming": True, "wp_idx": 0}
        log.write("/twin/state", float(t), {"vehicles": veh})
        log.write("/plant/truth", float(t), {"vehicles": {"0": [t + 0.5, 0.0, 40.0]}})
    log.write("/twin/prediction", 10.0, {"t": 10.0, "pos": {"60.0": {"0": [70.0, 0.0, 40.0]}}, "los": []})
    log.write("/twin/alert", 10.2, {"vid": 0, "event": "nonconforming", "phase": "en_route"})
    log.write("/twin/replan", 15.0, {"vid": 0, "status": "contingent", "hover_s": 0})
    log.close()
    return run, tmp_path / "worlds"


# ------------------------------------------------------------------ worlds
def test_px4_world_has_px4_systems(tiny_air):
    root = ET.fromstring(world_sdf(tiny_air, px4=True, real_time_factor=2))
    w = root.find("world")
    names = {p.get("name").rsplit("::", 1)[-1] for p in w.findall("plugin")}
    assert names == set(PX4_GZ_SYSTEMS)
    files = {p.get("filename") for p in w.findall("plugin")}
    assert "gz-sim-navsat-system" in files and "gz-sim-air-pressure-system" in files
    assert w.find("physics").get("type") == "ode"
    assert float(w.find("physics/max_step_size").text) == 0.004
    assert float(w.find("physics/real_time_factor").text) == 2
    assert w.find("magnetic_field") is not None and w.find("atmosphere") is not None
    assert float(w.find("spherical_coordinates/latitude_deg").text) == pytest.approx(52.0)
    assert w.find("model[@name='bldg_1']/link/collision") is not None


def test_view_world_unchanged_systems(tiny_air):
    w = ET.fromstring(world_sdf(tiny_air)).find("world")
    files = [p.get("filename") for p in w.findall("plugin")]
    assert files == ["gz-sim-physics-system", "gz-sim-user-commands-system", "gz-sim-scene-broadcaster-system",
                     "gz-sim-sensors-system", "gz-sim-imu-system", "gz-sim-navsat-system"]


def test_world_mesh(tiny_air):
    v, t = world_mesh(tiny_air)
    assert v.shape[1] == 3 and t.shape[1] == 3 and t.max() < len(v)
    assert v[:, 2].max() == pytest.approx(15.0)
    assert len(t) == 8 + 2                          # 4 walls x 2 triangles + a square roof


# ------------------------------------------------------------------ models and cameras
def test_marker_sdf_valid():
    m = ET.fromstring(drone_marker_sdf("veh_3_ok", (40, 180, 80), scale=4)).find("model")
    assert m.get("name") == "veh_3_ok" and m.find("static").text == "false"
    assert m.find("link/gravity").text == "false" and m.find("link/collision") is None
    c = ET.fromstring(camera_rig_sdf("cam", "/swarm/camera", 640, 360, 10)).find("model/link/sensor")
    assert c.get("type") == "camera" and c.find("topic").text == "/swarm/camera"
    assert c.find("camera/image/width").text == "640"


def _rotate(q, v):
    x, y, z, w = q
    u = np.array([x, y, z])
    v = np.asarray(v, float)
    return 2 * np.dot(u, v) * u + (w * w - np.dot(u, u)) * v + 2 * w * np.cross(u, v)


@pytest.mark.parametrize("eye,target", [((0, 0, 100), (50, 50, 0)), ((-300, 10, 40), (0, 0, 0)), ((5, 5, 5), (5, -20, 5))])
def test_look_at_points_camera_axis_at_target(eye, target):
    pos, rpy = look_at(eye, target)
    fwd = _rotate(rpy_to_quat(*rpy), (1, 0, 0))
    d = np.subtract(target, eye) / np.linalg.norm(np.subtract(target, eye))
    assert np.allclose(fwd, d, atol=1e-9)


def test_rpy_to_quat_unit_and_yaw():
    q = rpy_to_quat(0.1, -0.4, 2.0)
    assert np.linalg.norm(q) == pytest.approx(1.0)
    assert np.allclose(_rotate(rpy_to_quat(0, 0, math.pi / 2), (1, 0, 0)), (0, 1, 0), atol=1e-12)


def test_overview_and_follow_poses(tiny_air):
    pos, rpy = overview_pose(tiny_air.bounds)
    assert pos[0] < tiny_air.bounds[0] + 200 and pos[2] > 100 and rpy[1] > 0     # above, looking down
    pos, rpy = follow_pose((0, 0, 40), (10, 0, 0))
    assert pos[0] < 0 and pos[2] > 40 and abs(rpy[2]) < 1e-9                       # behind, facing +x


# ------------------------------------------------------------------ scene
def test_scene_interpolates_and_judges(run_dir):
    run, worlds = run_dir
    sc = Scene.from_run(run, worlds)
    assert sc.span() == (0.0, 20.0)
    f = sc.frame(4.5)
    assert set(f.vehicles) == {0, 1}
    assert np.allclose(f.vehicles[0].pos, [4.5, 0, 40]) and np.allclose(f.vehicles[0].vel, [1, 0, 0])
    assert f.vehicles[0].conforming and np.allclose(f.truth[0], [5.0, 0, 40])
    f = sc.frame(12.0)
    assert not f.vehicles[0].conforming and f.nonconforming == 1
    assert 1 not in f.vehicles                              # vehicle 1's last state was at 5 s: gone
    assert np.allclose(f.predicted[60.0][0], [70, 0, 40])
    assert [e.kind for e in f.events] == ["alert"] and f.events[0].warn     # events since the 4.5 s frame


def test_scene_intents_in_force(run_dir):
    run, worlds = run_dir
    sc = Scene.from_run(run, worlds)
    b = sc.intents_at(10.0)                                  # version 0: both boxes (t in [8, 12]); vehicle 1's box
    assert sorted((x.vid, x.certified) for x in b) == [(0, True), (0, True), (1, True)]
    b = sc.intents_at(20.0)                                  # version 0 withdrawn at 15 s, the replan is uncertified
    assert sorted((x.vid, x.certified, x.status) for x in b) == [(0, False, "contingent"), (1, True, "accepted")]
    x = next(x for x in b if x.vid == 0)
    assert np.allclose(x.lo, [10, 0, 35]) and np.allclose(x.size, [100, 10, 10]) and np.allclose(x.center, [60, 5, 40])
    assert sc.times(5.0) == [0.0, 5.0, 10.0, 15.0, 20.0]
    assert sc.times(5.0, start=7, end=16) == [7.0, 12.0]


# ------------------------------------------------------------------ Gazebo viewer (recorded calls)
def test_gz_view_spawns_parks_and_forgets(run_dir):
    run, worlds = run_dir
    sc = Scene.from_run(run, worlds)
    rc = RecordingClient()
    view = GzView(rc, ViewOptions(forget_s=3.0))
    view.apply(sc.frame(2.0))
    assert rc.alive == {"veh_0_ok", "veh_0_nc", "veh_1_ok", "veh_1_nc"}
    poses = dict(next(c for c in reversed(rc.calls) if c[0] == "set_poses")[1])
    assert poses["veh_0_ok"] == [2.0, 0.0, 40.0] and poses["veh_0_nc"][2] == PARK_Z
    view.apply(sc.frame(5.0))                                # vehicle 1's last state
    view.apply(sc.frame(7.0))                                # vehicle 1 absent for 2 s < forget_s: parked
    poses = dict(next(c for c in reversed(rc.calls) if c[0] == "set_poses")[1])
    assert poses["veh_1_ok"][2] == PARK_Z and poses["veh_1_nc"][2] == PARK_Z
    assert ("remove", "veh_1_ok") not in rc.calls
    view.apply(sc.frame(11.0))                               # vehicle 0 non-conforming: red one in place
    poses = dict(next(c for c in reversed(rc.calls) if c[0] == "set_poses")[1])
    assert poses["veh_0_nc"] == [11.0, 0.0, 40.0] and poses["veh_0_ok"][2] == PARK_Z
    assert ("remove", "veh_1_ok") in rc.calls                # absent for 6 s > forget_s: removed
    assert "veh_1_ok" not in poses and rc.alive == {"veh_0_ok", "veh_0_nc"}


def test_gz_view_marker_ids_are_reused_and_deleted(run_dir):
    run, worlds = run_dir
    sc = Scene.from_run(run, worlds)
    rc = RecordingClient()
    view = GzView(rc, ViewOptions())
    view.apply(sc.frame(10.0))
    first = [m for m in rc.calls[-1][1] if m["ns"] == "intents"]
    assert [m["id"] for m in first] == [1, 2, 3]
    view.apply(sc.frame(20.0))
    mk = [m for m in rc.calls[-1][1] if m["ns"] == "intents"]
    assert [(m["id"], m.get("action", "add")) for m in mk] == [(1, "add"), (2, "add"), (3, "delete")]
    labels = [m for m in rc.calls[-1][1] if m["ns"] == "labels"]
    assert labels[0]["text"] == "0 !"


def test_frame_markers_options(run_dir):
    run, worlds = run_dir
    f = Scene.from_run(run, worlds).frame(10.0)
    ns = {m["ns"] for m in frame_markers(f, ViewOptions())}
    assert ns == {"intents", "truth", "predicted", "predicted_path", "labels"}
    none = frame_markers(f, ViewOptions(intents=False, truth=False, predictions=False, labels=False))
    assert none == []
    f20 = Scene.from_run(run, worlds).frame(20.0)
    assert len([m for m in frame_markers(f20, ViewOptions(certified_only=True)) if m["ns"] == "intents"]) == 1


def test_gz_view_dump_cli(run_dir, tmp_path):
    run, worlds = run_dir
    out = tmp_path / "frames.jsonl"
    r = subprocess.run([sys.executable, str(ROOT / "scripts/gz_view.py"), str(run), "--world-dir", str(worlds),
                        "--dump", str(out), "--every", "5", "--quiet"], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    calls = [json.loads(line) for line in out.read_text().splitlines()]
    assert sorted({c["t"] for c in calls}) == [0.0, 5.0, 10.0, 15.0, 20.0]
    assert sum(c["call"] == "create" for c in calls) == 4


# ------------------------------------------------------------------ live link
def test_version_stream_resends_closed_versions():
    vs = VersionStream()
    vers = [{"vid": 0, "valid_from": 0.0, "valid_to": None, "int": [], "mission": {"big": "not sent"}}]
    out = vs.changed(vers)
    assert len(out) == 1 and "mission" not in out[0]
    assert vs.changed(vers) == []
    vers[0]["valid_to"] = 9.0
    vers.append({"vid": 1, "valid_from": 9.0, "valid_to": None, "int": []})
    out = vs.changed(vers)
    assert [(v["vid"], v["valid_to"]) for v in out] == [(0, 9.0), (1, None)]


def test_live_tap_reproduces_offline_scene(run_dir, tiny_air):
    """Messages through the tap -> JSON -> a live Scene give the same frame as the finished run."""
    run, worlds = run_dir
    offline = Scene.from_run(run, worlds)
    versions = json.loads((run / "intents.json").read_text())["versions"]

    class FakeTwin:
        intent_versions = versions
    sent = []
    tap = GzLiveTap(sent.append, FakeTwin())
    from swarm_sim.twin.log import read_log
    for topic, t, msg in read_log(run / "twin_log.mcap"):
        tap(topic, t, msg)
    live = Scene(tiny_air)
    for data in sent:
        live.feed(*decode(data))
    assert sum(decode(d)[0] == INTENT_TOPIC for d in sent) == 3
    for t in (3.0, 10.0, 18.5):
        a, b = offline.frame(t, since=-1), live.frame(t, since=-1)
        assert a.vehicles.keys() == b.vehicles.keys()
        assert all(np.allclose(a.vehicles[k].pos, b.vehicles[k].pos) for k in a.vehicles)
        assert len(a.intents) == len(b.intents) and len(a.events) == len(b.events)
    assert decode(encode("/x", 1.5, {"a": np.float64(2.0)})) == ("/x", 1.5, {"a": 2.0})


def test_twin_log_taps(tmp_path):
    got = []
    log = TwinLog(tmp_path / "l")
    log.taps.append(lambda *a: got.append(a))
    log.write("/twin/state", 1.0, {"vehicles": {}})
    log.close()
    assert got == [("/twin/state", 1.0, {"vehicles": {}})]


# ------------------------------------------------------------------ Foxglove
def test_foxglove_export_valid(run_dir, tmp_path):
    from mcap.reader import make_reader
    from swarm_sim.viz.foxglove import export
    run, worlds = run_dir
    out = tmp_path / "fg.mcap"
    counts = export(Scene.from_run(run, worlds), out, every=1.0)
    assert counts == {"frames": 21, "events": 2}
    try:
        import jsonschema
    except ImportError:
        jsonschema = None
    topics, schemas = {}, {}
    with open(out, "rb") as f:
        for s, ch, m in make_reader(f).iter_messages():
            topics[ch.topic] = topics.get(ch.topic, 0) + 1
            schemas[ch.topic] = s.name
            msg = json.loads(m.data)
            if jsonschema is not None and s.name.startswith("foxglove."):
                jsonschema.validate(msg, json.loads(s.data))
            if ch.topic == "/metrics" and msg["t"] == 12.0:
                assert msg["flying"] == 1 and msg["nonconforming"] == 1
    assert topics == {"/world": 1, "/scene": 21, "/metrics": 21, "/events": 2}
    assert schemas["/scene"] == "foxglove.SceneUpdate" and schemas["/events"] == "foxglove.Log"


# ------------------------------------------------------------------ recording, clock, PX4 helpers
def test_image_rgb_strips_row_padding():
    w, h, step = 3, 2, 12
    rows = [bytes(range(r * 20, r * 20 + 9)) + b"\xff\xff\xff" for r in range(h)]
    assert image_rgb(w, h, step, b"".join(rows)) == b"".join(r[:9] for r in rows)
    assert image_rgb(w, h, 9, bytes(18)) == bytes(18)
    cmd = ffmpeg_cmd("o.mp4", 640, 360, 10)
    assert cmd[cmd.index("-s") + 1] == "640x360" and cmd[-1] == "o.mp4"


def test_parse_clock_text():
    text = """header {
  stamp { sec: 3 }
}
real {
  sec: 5
  nsec: 1000
}
sim {
  sec: 4
  nsec: 500000000
}

sim {
  nsec: 4000000
}
sim {
  sec: 7
}
"""
    assert list(parse_clock_text(text.splitlines())) == pytest.approx([4.5, 0.004, 7.0])


def test_pad_free():
    pad = (0.0, 0.0, 0.0)
    assert pad_free(pad, {}, 4.0)
    assert not pad_free(pad, {1: (1.0, 1.0, 0.2)}, 4.0)              # standing on it
    assert not pad_free(pad, {1: (0.0, 2.0, 3.0)}, 4.0)              # climbing out, still low
    assert pad_free(pad, {1: (0.0, 2.0, 6.0)}, 4.0)                  # above 5 m
    assert pad_free(pad, {1: (8.0, 0.0, 0.0)}, 4.0)                  # the next pad


def test_instance_env_and_px4_paths(tmp_path):
    from swarm_sim.gz.launch import px4_gz_env, server_cmd
    env = px4_gz_env(tmp_path)
    assert env["PX4_GZ_MODELS"].endswith("Tools/simulation/gz/models")
    assert env["GZ_SIM_SYSTEM_PLUGIN_PATH"].endswith("build/px4_sitl_default/src/modules/simulation/gz_plugins")
    assert "GZ_SIM_SERVER_CONFIG_PATH" not in env
    e = instance_env(2, (10.0, -4.5, 0.2), "gz_x500", 1.5, env)
    assert e["PX4_GZ_MODEL_POSE"] == "10.000,-4.500,0.200" and e["PX4_GZ_STANDALONE"] == "1"
    assert e["PX4_SIM_MODEL"] == "gz_x500" and e["PX4_SIM_SPEED_FACTOR"] == "1.5"
    assert server_cmd("w.sdf", run=False)[:4] == ["gz", "sim", "-s", "-v1"] and "-r" not in server_cmd("w.sdf", run=False)


# ------------------------------------------------------------------ opt-in: real Gazebo
@pytest.mark.skipif(os.environ.get("RUN_GZ") != "1" or shutil.which("gz") is None,
                    reason="RUN_GZ=1 and Gazebo (service gz) needed")
def test_gz_world_drives_vehicles(run_dir, tmp_path, tiny_air):
    from swarm_sim.gz import launch
    from swarm_sim.gz.transport import Gz, load_bindings
    if load_bindings() is None:
        pytest.skip("gz-transport Python bindings not installed")
    run, worlds = run_dir
    wpath = tmp_path / "tiny.sdf"
    wpath.write_text(world_sdf(tiny_air))
    procs = launch.start(wpath, "tiny", run=True, logdir=tmp_path)
    try:
        gz = Gz("tiny")
        view = GzView(gz, ViewOptions())
        sc = Scene.from_run(run, worlds)
        for t in (1.0, 2.0, 11.0):
            view.apply(sc.frame(t))
        r = subprocess.run(["gz", "model", "--list"], capture_output=True, text=True, timeout=20)
        assert "veh_0_ok" in r.stdout and "veh_0_nc" in r.stdout
        view.clear()
    finally:
        launch.stop(procs)


@pytest.mark.skipif(os.environ.get("RUN_PX4GZ") != "1", reason="RUN_PX4GZ=1 and the px4gz image needed")
def test_px4_gazebo_twin_flies(tmp_path):
    world = ROOT / "data/worlds/mini_town"
    if not world.exists():                               # data/ is not in git: build the world from the test fixture
        r = subprocess.run([sys.executable, str(ROOT / "scripts/osm_to_world.py"), "--osm",
                            str(ROOT / "tests/fixtures/mini_town.osm"), "--name", "mini_town", "--hubs",
                            str(ROOT / "configs/hubs_mini_town.yaml"), "--out", str(tmp_path / "worlds")],
                           capture_output=True, text=True)
        assert r.returncode == 0, r.stderr[-2000:]
        world = tmp_path / "worlds/mini_town"
    out = tmp_path / "runs"
    r = subprocess.run([sys.executable, str(ROOT / "scripts/run_twin.py"), "--world", str(world), "--plant", "px4gz",
                        "--missions", "1", "--out", str(out)], capture_output=True, text=True, timeout=3600)
    assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-3000:]
    rep = json.loads(next(out.glob("*/report.json")).read_text())
    assert rep["plant_truth"]["simulator"] == "gazebo" and rep["plant_truth"]["completed"] == 1


def test_px4gz_lifecycle_shares_pads(tiny_air):
    """Two flights from one pad: the second x500 spawns only once the first has climbed away; both are removed after
    landing; a cancelled request never spawns. PX4, MAVSDK and Gazebo are replaced by fakes; the clock is driven."""
    import asyncio
    import threading
    import time as _time
    from swarm_sim.missions import Mission, Waypoint
    from swarm_sim.sim import SimParams
    from swarm_sim.twin.px4_gz import Px4GzPlant
    from swarm_sim.vehicle import VehicleParams

    pad = [0.0, 0.0, 0.0]
    reqs = [Mission(i, "delivery", 2, r, "H", 0, [Waypoint(50, 0, 40)], meta={"start": pad, "pad": pad})
            for i, r in enumerate((50.0, 60.0, 70.0))]
    events = []

    class Fake(Px4GzPlant):
        poll_s = 0.0005

        def _launch(self):
            self.gz_env = {}

        def _spawn_px4(self, k, pos):
            events.append((round(self.t_plant), "spawn", k))

        def _despawn(self, k):
            events.append((round(self.t_plant), "despawn", k))
            with self.lock:
                self.spawned.pop(k, None)

        async def _connect(self, k):
            return None

        async def _vehicle(self, k, d):
            with self.lock:
                self.last_pos[k] = np.array(pad)
            while self.t_plant < self.requests[k].release_t:
                await asyncio.sleep(0.005)
            while self.t_plant < self.requests[k].release_t + 3:      # climbing through the first 5 m
                await asyncio.sleep(0.005)
            with self.lock:
                self.last_pos[k] = np.array([0.0, 0.0, 20.0])
                self.state[k] = "flying"
            while self.t_plant < self.requests[k].release_t + 30:
                await asyncio.sleep(0.005)
            with self.lock:
                self.state[k] = "landed"

    plant = Fake(reqs, tiny_air, VehicleParams(), SimParams(), spawn_lead_s=45, log=lambda *_: None)
    plant.cancel(2)                                       # rejected before its spawn time (25 s)

    def drive():
        for i in range(0, 1200):
            with plant.lock:
                plant.t_plant = i * 0.1
            _time.sleep(0.004)
    th = threading.Thread(target=drive)
    th.start()
    th.join()
    _time.sleep(0.2)
    plant.loop.call_soon_threadsafe(plant.loop.stop)
    assert not plant.err
    assert events[0][1:] == ("spawn", 0) and abs(events[0][0] - 5) <= 1
    spawn1 = next(t for t, e, k in events if e == "spawn" and k == 1)
    assert abs(spawn1 - 53) <= 1                          # waited for vehicle 0 to climb away (50 s + 3 s), not 15 s
    assert sorted(k for _t, e, k in events if e == "despawn") == [0, 1]
    assert all(k != 2 for _t, _e, k in events)
    plant.t_end = 0.0
    assert not plant.spawned and plant.advance() is False       # finished only once every vehicle is removed
    plant.spawned[5] = (0.0, 0.0, 0.0)
    assert plant.advance() is True


# ------------------------------------------------------------------ live link: loop, pacing, marker fallback
class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


def test_run_live_draws_and_exits_when_idle(run_dir, tiny_air):
    """The whole live path minus gz-transport: tap -> JSON -> feed -> Scene -> GzView, with a fake clock."""
    from swarm_sim.gz.live import run_live
    from swarm_sim.twin.log import read_log
    run, worlds = run_dir
    versions = json.loads((run / "intents.json").read_text())["versions"]

    class FakeTwin:
        intent_versions = versions
    wire = []
    tap = GzLiveTap(wire.append, FakeTwin())
    for topic, t, msg in read_log(run / "twin_log.mcap"):
        tap(topic, t, msg)

    clock = _Clock()

    class Feed:                                    # delivers the wire in 3 bursts, 1 s apart, then nothing
        def __init__(self):
            self.chunks = [wire[: len(wire) // 3], wire[len(wire) // 3: 2 * len(wire) // 3], wire[2 * len(wire) // 3:]]
            self.next_t = 1.0

        def drain(self):
            if self.chunks and clock.t >= self.next_t:
                self.next_t += 1.0
                return [decode(d) for d in self.chunks.pop(0)]
            return []
    rc = RecordingClient()
    lines, frames = [], []
    st = run_live(Feed(), Scene(tiny_air), GzView(rc, ViewOptions()), fps=10, idle_exit_s=5, first_wait_s=60,
                  status_every_s=2, on_frame=frames.append, echo=lines.append, clock=clock, sleep=clock.sleep)
    assert st.messages == len(wire) and st.frames == 3 and st.sim_t == 20.0
    # frames are drawn at each burst's latest state (~6, 13, 20 s): 2 boxes in force at most (3 only in 8..12 s)
    assert st.max_vehicles == 2 and st.max_intents == 2 and st.max_nonconforming == 1
    assert st.exit_reason == "no message for 5 s" and 8.0 <= clock.t <= 8.3
    assert lines and lines[0].startswith("live t=")
    assert any(c[0] == "create" and c[1] == "veh_0_ok" for c in rc.calls)
    assert sum(len(f.events) for f in frames) == 2          # the alert and the replan, each once


def test_run_live_gives_up_without_messages():
    from swarm_sim.gz.live import run_live
    clock = _Clock()

    class Empty:
        def drain(self):
            return []
    st = run_live(Empty(), Scene(), GzView(RecordingClient()), fps=10, idle_exit_s=5, first_wait_s=3,
                  status_every_s=0, echo=lambda *_: None, clock=clock, sleep=clock.sleep)
    assert st.messages == 0 and st.exit_reason == "no message within 3 s"


def test_view_markers_back_off_without_gui(run_dir):
    run, worlds = run_dir
    sc = Scene.from_run(run, worlds)
    clock = _Clock()

    class NoGui(RecordingClient):
        gui = False

        def markers(self, markers):
            super().markers(markers)
            return self.gui
    rc = NoGui()
    view = GzView(rc, ViewOptions(), clock=clock)
    view.apply(sc.frame(2.0))
    view.apply(sc.frame(3.0))
    assert sum(c[0] == "markers" for c in rc.calls) == 1        # failed once, then skipped
    rc.gui = True
    clock.t = GzView.MARKER_RETRY_S + 1
    view.apply(sc.frame(4.0))
    assert sum(c[0] == "markers" for c in rc.calls) == 2 and view.marker_ids


def test_view_no_vehicles_draws_markers_only(run_dir):
    run, worlds = run_dir
    rc = RecordingClient()
    GzView(rc, ViewOptions(vehicles=False)).apply(Scene.from_run(run, worlds).frame(10.0))
    assert [c[0] for c in rc.calls] == ["markers"] and not rc.alive


def test_paced_plant_holds_sim_to_pace():
    from swarm_sim.twin.telemetry import PacedPlant
    clock = _Clock()

    class Fast:
        name = "fast"

        def __init__(self):
            self.t = 0.0

        def now(self):
            return self.t

        def advance(self):
            self.t += 0.1
            return self.t < 100
    p = PacedPlant(Fast(), pace=4, sleep=clock.sleep, clock=clock)
    while p.advance():
        pass
    assert p.name == "fast" and p.now() == pytest.approx(100.0, abs=0.2)
    assert clock.t == pytest.approx((p.now() - 0.1) / 4, abs=1e-6)     # 100 sim s at 4x = 25 wall s


def test_live_demo_script_parses():
    r = subprocess.run(["bash", "-n", str(ROOT / "scripts/live_demo.sh")], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


def test_marker_ids_never_zero(run_dir):
    """Gazebo treats marker id 0 as 'new marker', so id 0 would leave a copy every frame: ids go out from 1, and the
    same markers keep the same ids from frame to frame."""
    run, worlds = run_dir
    sc = Scene.from_run(run, worlds)
    rc = RecordingClient()
    view = GzView(rc, ViewOptions())
    view.apply(sc.frame(10.0))
    a = {(m["ns"], m["id"]) for m in rc.calls[-1][1]}
    view.apply(sc.frame(11.0))
    b = {(m["ns"], m["id"]) for m in rc.calls[-1][1] if m.get("action") != "delete"}
    assert min(i for _ns, i in a | b) == 1
    assert a == b                                            # same set of markers: updated in place, none added
