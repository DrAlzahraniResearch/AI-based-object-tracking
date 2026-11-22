"""Scene frames as Foxglove messages, in one MCAP file (scripts/foxglove_export.py).

Channels (JSON encoding, Foxglove's own schemas, so the 3D, Log and Plot panels work with no setup):

  /world    foxglove.SceneUpdate   buildings (one triangle mesh), hubs and pads, no-fly volumes; once, at the start
  /scene    foxglove.SceneUpdate   vehicles (green / red), plant truth, 60 s predictions, intent boxes in force
  /events   foxglove.Log           alerts, replans, decisions, arrivals, calibration (WARNING when they matter)
  /metrics  JSON                   flying, non-conforming, intent boxes in force (certified / not), for Plot panels

Everything is in frame "world" (ENU metres, the airspace's origin). Timestamps are sim time from 0 s.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from ..osm import Airspace
from .scene import NC_RGB, OK_RGB, PRED_RGB, TRUTH_RGB, VEHICLE_PALETTE, Frame, Scene, world_mesh

SCHEMAS = Path(__file__).with_name("schemas")
FRAME_ID = "world"
IDENTITY = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}


def _ts(t: float) -> dict:
    sec = int(math.floor(t))
    return {"sec": sec, "nsec": int(round((t - sec) * 1e9)) % 1_000_000_000}


def _color(rgb, a=1.0) -> dict:
    return {"r": rgb[0] / 255.0, "g": rgb[1] / 255.0, "b": rgb[2] / 255.0, "a": a}


def _vec(p) -> dict:
    return {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])}


def _pose(p) -> dict:
    return {"position": _vec(p), "orientation": IDENTITY}


def entity(t: float, eid: str, **prims) -> dict:
    e = {"timestamp": _ts(t), "frame_id": FRAME_ID, "id": eid, "lifetime": {"sec": 0, "nsec": 0},
         "frame_locked": False, "metadata": []}
    for k in ("arrows", "cubes", "spheres", "cylinders", "lines", "triangles", "texts", "models"):
        e[k] = prims.get(k, [])
    return e


def world_update(air: Airspace, t: float = 0.0) -> dict:
    verts, tris = world_mesh(air)
    ents = [entity(t, "buildings", triangles=[{
        "pose": _pose((0, 0, 0)), "points": [_vec(v) for v in verts], "color": _color((175, 175, 185)),
        "colors": [], "indices": [int(i) for i in tris.reshape(-1)]}])]
    pads, labels = [], []
    for h in air.hubs:
        for px, py in h.pad_positions():
            pads.append({"pose": _pose((px, py, h.elevation + 0.05)), "size": _vec((4, 4, 0.1)),
                         "color": _color((240, 190, 30))})
        labels.append({"pose": _pose((h.x, h.y, h.elevation + 12)), "billboard": True, "font_size": 14,
                       "scale_invariant": True, "color": _color((40, 120, 200)), "text": h.name})
    ents.append(entity(t, "hubs", cubes=pads, texts=labels))
    lines = []
    for nf in air.no_fly:
        ring = np.array(nf.polygon, float)
        top = min(nf.ceiling, air.ceiling_m)
        for z in (nf.floor, top):
            lines.append({"type": 1, "pose": _pose((0, 0, 0)), "thickness": 2, "scale_invariant": True,
                          "points": [_vec((x, y, z)) for x, y in ring[:-1]], "color": _color((220, 60, 60)),
                          "colors": [], "indices": []})
    ents.append(entity(t, "no_fly", lines=lines))
    return {"deletions": [], "entities": ents}


def scene_update(f: Frame, horizon: float = 60.0) -> dict:
    t = f.t
    veh = [{"pose": _pose(v.pos), "size": _vec((6, 6, 6)), "color": _color(OK_RGB if v.conforming else NC_RGB)}
           for v in f.vehicles.values()]
    lab = [{"pose": _pose(v.pos + np.array([0, 0, 8])), "billboard": True, "font_size": 12, "scale_invariant": True,
            "color": _color(OK_RGB if v.conforming else NC_RGB), "text": str(v.vid)} for v in f.vehicles.values()]
    tru = [{"pose": _pose(p), "size": _vec((2.5, 2.5, 2.5)), "color": _color(TRUTH_RGB, 0.8)} for p in f.truth.values()]
    pred = [{"pose": _pose(p), "size": _vec((3, 3, 3)), "color": _color(PRED_RGB, 0.9)}
            for p in (f.predicted.get(horizon) or {}).values()]
    boxes = [{"pose": _pose(b.center), "size": _vec(b.size),
              "color": _color(VEHICLE_PALETTE[b.vid % len(VEHICLE_PALETTE)], 0.18 if b.certified else 0.07)}
             for b in f.intents]
    return {"deletions": [], "entities": [
        entity(t, "vehicles", spheres=veh, texts=lab), entity(t, "truth", spheres=tru),
        entity(t, "predicted", spheres=pred), entity(t, "intents", cubes=boxes)]}


def metrics(f: Frame) -> dict:
    cert = sum(b.certified for b in f.intents)
    return {"t": round(f.t, 3), "flying": len(f.vehicles), "nonconforming": f.nonconforming,
            "intents_certified": cert, "intents_uncertified": len(f.intents) - cert}


def export(scene: Scene, out: str | Path, every: float = 1.0, start: float | None = None,
           end: float | None = None) -> dict:
    from mcap.writer import CompressionType, Writer
    out = Path(out)
    counts = {"frames": 0, "events": 0}
    with open(out, "wb") as fh:
        w = Writer(fh, compression=CompressionType.ZSTD if _has_zstd() else CompressionType.NONE)
        w.start(profile="", library="civilian-swarm-viz")

        def schema(name, file=None):
            data = (SCHEMAS / file).read_bytes() if file else b'{"type":"object"}'
            return w.register_schema(name=name, encoding="jsonschema", data=data)
        scene_schema = schema("foxglove.SceneUpdate", "SceneUpdate.json")
        ch = {
            "/world": w.register_channel("/world", "json", scene_schema),
            "/scene": w.register_channel("/scene", "json", scene_schema),
            "/events": w.register_channel("/events", "json", schema("foxglove.Log", "Log.json")),
            "/metrics": w.register_channel("/metrics", "json", schema("swarm.Metrics")),
        }

        def put(topic, t, msg):
            ns = int(round(t * 1e9))
            w.add_message(ch[topic], log_time=ns, publish_time=ns, data=json.dumps(msg, separators=(",", ":")).encode())

        times = scene.times(every, start, end)
        if times:
            put("/world", times[0], world_update(scene.air, times[0]))
        prev = None
        for t in times:
            f = scene.frame(t, since=prev if prev is not None else -math.inf)
            prev = t
            put("/scene", t, scene_update(f))
            put("/metrics", t, metrics(f))
            for e in f.events:
                put("/events", e.t, {"timestamp": _ts(e.t), "level": 3 if e.warn else 2, "message": e.text,
                                     "name": e.kind, "file": "", "line": 0})
                counts["events"] += 1
            counts["frames"] += 1
        w.finish()
    return counts


def _has_zstd() -> bool:
    try:
        import zstandard  # noqa: F401
        return True
    except ImportError:
        return False
