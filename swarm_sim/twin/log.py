"""Twin log: MCAP (the rosbag2 default storage format) with JSON messages, one channel per topic.

Topics: /twin/telemetry (every sample, per tick), /twin/state (1 Hz estimates + conformance),
/twin/decision, /twin/alert, /twin/replan, /twin/calibration, /twin/prediction, /plant/truth (sim plants).
Readable with `mcap` (Python/CLI), Foxglove, or `scripts/twin_view.py` (Rerun). Falls back to JSON Lines
when the `mcap` package is missing.
"""
from __future__ import annotations

import json
from pathlib import Path


class TwinLog:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.channels: dict[str, int] = {}
        self.taps: list = []                      # callables(topic, t, msg): live consumers (swarm_sim/gz/live.py)
        try:
            from mcap.writer import CompressionType, Writer
            self.f = open(self.path.with_suffix(".mcap"), "wb")
            try:
                import zstandard  # noqa: F401
                comp = CompressionType.ZSTD
            except ImportError:
                comp = CompressionType.NONE
            self.w = Writer(self.f, compression=comp)
            self.w.start(profile="", library="civilian-swarm-twin")
            self.schema = self.w.register_schema(name="json", encoding="jsonschema", data=b'{"type":"object"}')
            self.kind = "mcap"
        except ImportError:
            self.f = open(self.path.with_suffix(".jsonl"), "w")
            self.w = None
            self.kind = "jsonl"
        self.file = Path(self.f.name)

    def write(self, topic: str, t: float, msg: dict):
        for tap in self.taps:
            tap(topic, t, msg)
        ns = int(round(t * 1e9))
        if self.w is None:
            self.f.write(json.dumps({"topic": topic, "t": t, "msg": msg}) + "\n")
            return
        ch = self.channels.get(topic)
        if ch is None:
            ch = self.channels[topic] = self.w.register_channel(topic=topic, message_encoding="json",
                                                                schema_id=self.schema)
        self.w.add_message(ch, log_time=ns, publish_time=ns, data=json.dumps(msg).encode())

    def close(self):
        if self.w is not None:
            self.w.finish()
        self.f.close()


def read_log(path: str | Path, topics: list[str] | None = None):
    """Yield (topic, t, msg) from a twin log (.mcap or .jsonl)."""
    path = Path(path)
    if path.suffix == ".jsonl":
        with open(path) as f:
            for line in f:
                r = json.loads(line)
                if topics is None or r["topic"] in topics:
                    yield r["topic"], r["t"], r["msg"]
        return
    from mcap.reader import make_reader
    with open(path, "rb") as f:
        for _schema, ch, m in make_reader(f).iter_messages(topics=topics):
            yield ch.topic, m.log_time / 1e9, json.loads(m.data)
