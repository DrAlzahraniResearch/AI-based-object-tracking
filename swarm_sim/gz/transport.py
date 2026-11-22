"""Talking to a running Gazebo: Python bindings when installed, the `gz` CLI otherwise.

Bindings: Gazebo Harmonic ships `gz.transport13` / `gz.msgs10` (apt: python3-gz-transport13 python3-gz-msgs10),
Ionic `gz.transport14` / `gz.msgs11`, Jetty unversioned `gz.transport` / `gz.msgs` (or 15 / 12). `load_bindings`
tries them in that order. The viewer (markers, video, live link) needs them; the PX4 plant can run on the CLI
alone (`gz service`, `gz topic -e`).

Services used (gz-sim UserCommands, SceneBroadcaster, MarkerManager in the GUI):
  /world/<w>/create (EntityFactory)  /world/<w>/remove (Entity)  /world/<w>/set_pose_vector (Pose_V)
  /world/<w>/control (WorldControl)  /world/<w>/scene/info       /marker_array (Marker_V, GUI only)
"""
from __future__ import annotations

import importlib
import subprocess
import threading
import time
from typing import Callable, Iterable

_VERSIONS = (("13", "10"), ("14", "11"), ("", ""), ("15", "12"))


def load_bindings():
    """(gz.transport module, gz.msgs package name) or None."""
    for tv, mv in _VERSIONS:
        try:
            tr = importlib.import_module(f"gz.transport{tv}")
            importlib.import_module(f"gz.msgs{mv}.stringmsg_pb2")
            return tr, f"gz.msgs{mv}"
        except ImportError:
            continue
    return None


# ------------------------------------------------------------------ CLI fallbacks
def cli_service(service: str, reqtype: str, reptype: str, req: str, timeout_ms: int = 3000) -> bool:
    try:
        r = subprocess.run(["gz", "service", "-s", service, "--reqtype", reqtype, "--reptype", reptype,
                            "--timeout", str(timeout_ms), "--req", req],
                           capture_output=True, text=True, timeout=timeout_ms / 1000 + 5)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return r.returncode == 0 and "data: true" in r.stdout


def world_ready(world: str) -> bool:
    try:
        r = subprocess.run(["gz", "service", "-i", "--service", f"/world/{world}/scene/info"],
                           capture_output=True, text=True, timeout=10)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return "Service providers" in (r.stdout + r.stderr)


def wait_for_world(world: str, timeout_s: float = 120.0, procs: Iterable[subprocess.Popen] = ()) -> None:
    t0 = time.monotonic()
    while not world_ready(world):
        for p in procs:
            if p.poll() is not None:
                raise RuntimeError(f"gz sim exited with code {p.returncode} before world '{world}' came up")
        if time.monotonic() - t0 > timeout_s:
            raise TimeoutError(f"Gazebo world '{world}' not up after {timeout_s:.0f} s")
        time.sleep(1.0)


def cli_remove_model(world: str, name: str) -> bool:
    return cli_service(f"/world/{world}/remove", "gz.msgs.Entity", "gz.msgs.Boolean", f'name: "{name}", type: MODEL')


def parse_clock_text(lines: Iterable[str]):
    """Sim times (s) from `gz topic -e -t /world/<w>/clock` text output. Protobuf text omits zero fields."""
    stack: list[str] = []
    sec = nsec = 0
    for raw in lines:
        line = raw.strip()
        if line.endswith("{"):
            stack.append(line[:-1].strip())
            if stack == ["sim"]:
                sec = nsec = 0
        elif line == "}":
            if stack == ["sim"]:
                yield sec + nsec * 1e-9
            if stack:
                stack.pop()
        elif stack == ["sim"] and ":" in line:
            k, v = (s.strip() for s in line.split(":", 1))
            if k == "sec":
                sec = int(v)
            elif k == "nsec":
                nsec = int(v)


class GzClock:
    """Gazebo sim time of a world, kept current by a background subscription (bindings, else the CLI)."""

    def __init__(self, world: str, on_time: Callable[[float], None] | None = None):
        self.world, self.t, self.on_time = world, 0.0, on_time
        self.proc = None
        self._stop = threading.Event()
        b = load_bindings()
        topic = f"/world/{world}/clock"
        if b is not None:
            tr, mp = b
            Clock = importlib.import_module(f"{mp}.clock_pb2").Clock
            self.node = tr.Node()
            self.node.subscribe(Clock, topic, lambda m: self._set(m.sim.sec + m.sim.nsec * 1e-9))
        else:
            self.proc = subprocess.Popen(["gz", "topic", "-e", "-t", topic], stdout=subprocess.PIPE,
                                         stderr=subprocess.DEVNULL, text=True, bufsize=1)
            threading.Thread(target=self._read_cli, daemon=True).start()

    def _set(self, t: float):
        self.t = t
        if self.on_time:
            self.on_time(t)

    def _read_cli(self):
        for t in parse_clock_text(self.proc.stdout):
            if self._stop.is_set():
                return
            self._set(t)

    def close(self):
        self._stop.set()
        if self.proc is not None:
            self.proc.terminate()


# ------------------------------------------------------------------ bindings client
class Gz:
    """A gz-transport node bound to one world. Raises RuntimeError without the Python bindings."""

    def __init__(self, world: str, timeout_ms: int = 5000):
        b = load_bindings()
        if b is None:
            raise RuntimeError("gz-transport Python bindings not found (Harmonic: apt install python3-gz-transport13 "
                               "python3-gz-msgs10; Docker: service gz)")
        self.tr, self.mp = b
        self.node = self.tr.Node()
        self.world, self.timeout = world, timeout_ms
        self._pubs: dict = {}

    def msg(self, module: str, cls: str):
        return getattr(importlib.import_module(f"{self.mp}.{module}_pb2"), cls)

    def _request(self, service: str, req, rep_cls) -> bool:
        ok, rep = self.node.request(service, req, type(req), rep_cls, self.timeout)
        return bool(ok and getattr(rep, "data", True))

    def _boolean(self):
        return self.msg("boolean", "Boolean")

    # ---------------------------------------------------------------- entities
    def create(self, sdf: str, name: str, pos=(0, 0, 0), quat=(0, 0, 0, 1)) -> bool:
        req = self.msg("entity_factory", "EntityFactory")()
        req.sdf, req.name, req.allow_renaming = sdf, name, False
        self._fill_pose(req.pose, pos, quat)
        return self._request(f"/world/{self.world}/create", req, self._boolean())

    def remove(self, name: str) -> bool:
        Entity = self.msg("entity", "Entity")
        req = Entity()
        req.name, req.type = name, Entity.MODEL
        return self._request(f"/world/{self.world}/remove", req, self._boolean())

    @staticmethod
    def _fill_pose(p, pos, quat):
        p.position.x, p.position.y, p.position.z = (float(v) for v in pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in quat)

    def set_poses(self, poses: list[tuple[str, tuple, tuple]]) -> bool:
        if not poses:
            return True
        req = self.msg("pose_v", "Pose_V")()
        for name, pos, quat in poses:
            p = req.pose.add()
            p.name = name
            self._fill_pose(p, pos, quat)
        return self._request(f"/world/{self.world}/set_pose_vector", req, self._boolean())

    # ---------------------------------------------------------------- world control
    def control(self, pause: bool | None = None, multi_step: int = 0) -> bool:
        req = self.msg("world_control", "WorldControl")()
        if pause is not None:
            req.pause = pause
        if multi_step:
            req.pause = True
            req.multi_step = int(multi_step)
        return self._request(f"/world/{self.world}/control", req, self._boolean())

    # ---------------------------------------------------------------- markers (rendered by the GUI only)
    def markers(self, markers: list[dict]) -> bool:
        if not markers:
            return True
        Marker = self.msg("marker", "Marker")
        req = self.msg("marker_v", "Marker_V")()
        for d in markers:
            m = req.marker.add()
            m.ns, m.id = d["ns"], int(d["id"])
            if d.get("action") == "delete":
                m.action = Marker.DELETE_MARKER
                continue
            m.action = Marker.ADD_MODIFY
            m.type = {"box": Marker.BOX, "sphere": Marker.SPHERE, "line_strip": Marker.LINE_STRIP,
                      "points": Marker.POINTS, "text": Marker.TEXT}[d["type"]]
            m.visibility = Marker.GUI
            self._fill_pose(m.pose, d.get("pos", (0, 0, 0)), d.get("quat", (0, 0, 0, 1)))
            sx, sy, sz = d.get("size", (1, 1, 1))
            m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
            r, g, b, a = d.get("rgba", (1, 1, 1, 1))
            for c in (m.material.ambient, m.material.diffuse):
                c.r, c.g, c.b, c.a = r, g, b, a
            for pt in d.get("points", ()):
                q = m.point.add()
                q.x, q.y, q.z = (float(v) for v in pt)
            if "text" in d:
                m.text = d["text"]
        return self._request("/marker_array", req, self._boolean())

    # ---------------------------------------------------------------- topics
    def subscribe(self, topic: str, module: str, cls: str, cb) -> bool:
        return self.node.subscribe(self.msg(module, cls), topic, cb)

    def publish_string(self, topic: str, data: str):
        pub = self._pubs.get(topic)
        if pub is None:
            StringMsg = self.msg("stringmsg", "StringMsg")
            pub = self._pubs[topic] = (self.node.advertise(topic, StringMsg), StringMsg)
        p, StringMsg = pub
        m = StringMsg()
        m.data = data
        p.publish(m)
