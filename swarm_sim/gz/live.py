"""The twin, live, on a gz-transport topic.

`GzLiveTap` is a `TwinLog` tap (run_twin.py --gz-publish): every logged message on the viewer's topics, plus each
new or closed plan version (`/twin/intent_version`), goes out as JSON in a gz.msgs.StringMsg on `/swarm/twin`.
`GzLiveFeed` subscribes and queues them for `swarm_sim.viz.scene.Scene.feed` (scripts/gz_view.py --live).
The twin still never reads Gazebo (decision S22): this is one-way, twin -> viewer.
"""
from __future__ import annotations

import json
import queue
import time
from dataclasses import dataclass

from ..viz.scene import EVENT_TOPICS, INTENT_TOPIC

TOPIC = "/swarm/twin"
LIVE_TOPICS = ("/twin/state", "/plant/truth", "/twin/prediction", *EVENT_TOPICS)
VERSION_KEYS = ("vid", "valid_from", "valid_to", "status", "int", "phase", "cert")


def encode(topic: str, t: float, msg: dict) -> str:
    return json.dumps({"topic": topic, "t": t, "msg": msg}, default=float)


def decode(data: str) -> tuple[str, float, dict]:
    d = json.loads(data)
    return d["topic"], float(d["t"]), d["msg"]


class VersionStream:
    """Plan versions not yet sent, or whose validity changed since they were sent."""

    def __init__(self):
        self.sent: dict[int, object] = {}

    def changed(self, versions: list[dict]) -> list[dict]:
        out = []
        for i, v in enumerate(versions):
            if self.sent.get(i, "unsent") != v.get("valid_to"):
                self.sent[i] = v.get("valid_to")
                out.append({k: v.get(k) for k in VERSION_KEYS})
        return out


class GzLiveTap:
    def __init__(self, publish, twin=None, topics=LIVE_TOPICS):
        """publish(str): e.g. `lambda s: gz.publish_string(TOPIC, s)`."""
        self.publish, self.twin, self.topics = publish, twin, set(topics)
        self.versions = VersionStream()

    def __call__(self, topic: str, t: float, msg: dict):
        if topic not in self.topics:
            return
        if topic == "/twin/state" and self.twin is not None:
            for v in self.versions.changed(self.twin.intent_versions):
                self.publish(encode(INTENT_TOPIC, t, v))
        self.publish(encode(topic, t, msg))


class GzLiveFeed:
    def __init__(self, gz, topic: str = TOPIC):
        self.q: queue.Queue = queue.Queue()
        gz.subscribe(topic, "stringmsg", "StringMsg", lambda m: self.q.put(m.data))

    def drain(self) -> list[tuple[str, float, dict]]:
        out = []
        while True:
            try:
                out.append(decode(self.q.get_nowait()))
            except queue.Empty:
                return out


@dataclass
class LiveStats:
    messages: int = 0
    frames: int = 0
    sim_t: float | None = None
    max_vehicles: int = 0
    max_intents: int = 0
    max_nonconforming: int = 0
    exit_reason: str = ""

    def line(self) -> str:
        t = "-" if self.sim_t is None else f"{self.sim_t:.0f} s"
        return (f"live t={t}: {self.messages} messages, {self.frames} frames, up to {self.max_vehicles} vehicles, "
                f"{self.max_intents} intent boxes, {self.max_nonconforming} non-conforming")


def run_live(feed, scene, view, fps: float = 10.0, idle_exit_s: float | None = None,
             first_wait_s: float | None = None, status_every_s: float | None = 10.0, on_frame=None,
             echo=print, clock=time.monotonic, sleep=time.sleep) -> LiveStats:
    """Draw the twin as its messages arrive. A frame is drawn whenever new messages came in, at the latest
    /twin/state time. Stops `idle_exit_s` after the last message (once there was one), or after `first_wait_s`
    without any message; otherwise runs until interrupted."""
    st = LiveStats()
    start = last_msg = last_status = clock()
    last_line = None
    while True:
        msgs = feed.drain()
        now = clock()
        for topic, t, msg in msgs:
            scene.feed(topic, t, msg)
            if topic == "/twin/state":
                st.sim_t = t if st.sim_t is None else max(st.sim_t, t)
        if msgs:
            st.messages += len(msgs)
            last_msg = now
            if st.sim_t is not None:
                f = scene.frame(st.sim_t)
                view.apply(f)
                st.frames += 1
                st.max_vehicles = max(st.max_vehicles, len(f.vehicles))
                st.max_intents = max(st.max_intents, len(f.intents))
                st.max_nonconforming = max(st.max_nonconforming, f.nonconforming)
                if on_frame is not None:
                    on_frame(f)
        if status_every_s and now - last_status >= status_every_s:
            last_status = now
            if st.line() != last_line:                # nothing new while idle: stay quiet
                last_line = st.line()
                echo(last_line)
        if st.messages and idle_exit_s is not None and now - last_msg > idle_exit_s:
            st.exit_reason = f"no message for {idle_exit_s:g} s"
            return st
        if not st.messages and first_wait_s is not None and now - start > first_wait_s:
            st.exit_reason = f"no message within {first_wait_s:g} s"
            return st
        sleep(1.0 / fps)
