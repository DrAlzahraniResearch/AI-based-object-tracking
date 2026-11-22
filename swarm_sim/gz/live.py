"""The twin, live, on a gz-transport topic.

`GzLiveTap` is a `TwinLog` tap (run_twin.py --gz-publish): every logged message on the viewer's topics, plus each
new or closed plan version (`/twin/intent_version`), goes out as JSON in a gz.msgs.StringMsg on `/swarm/twin`.
`GzLiveFeed` subscribes and queues them for `swarm_sim.viz.scene.Scene.feed` (scripts/gz_view.py --live).
The twin still never reads Gazebo (decision S22): this is one-way, twin -> viewer.
"""
from __future__ import annotations

import json
import queue

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
