"""Draw scene frames into a running Gazebo world.

Vehicles are models (server entities, so a camera sensor sees them too): each vehicle gets two markers, green
(conforming) and red (not), and the one not in use is parked below the ground, so conformance shows in recorded
video without changing materials at run time. A vehicle absent from the scene for `forget_s` is removed.

Intent boxes, predictions and plant truth are GUI markers (`/marker_array`): they show in `gz sim -g`, not in the
headless camera. Markers reuse ids frame to frame; ids no longer in use are deleted.

`client` is a `swarm_sim.gz.transport.Gz`, or any object with the same `create`, `remove`, `set_poses` and
`markers` methods (tests use a recorder).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np

from ..viz.scene import NC_RGB, OK_RGB, PRED_RGB, TRUTH_RGB, VEHICLE_PALETTE, Frame
from .models import drone_marker_sdf, rpy_to_quat

PARK_Z = -200.0
LINE_TYPES = ("line_strip", "points")


@dataclass
class ViewOptions:
    scale: float = 4.0            # vehicle marker size, x a 0.5 m quadrotor
    vehicles: bool = True         # False: markers only (the world already has the vehicles, e.g. PX4 x500s)
    intents: bool = True
    truth: bool = True
    predictions: bool = True
    labels: bool = True
    certified_only: bool = False
    forget_s: float = 20.0
    prediction_horizon: float = 60.0


def _rgba(rgb, a=1.0):
    return tuple(c / 255.0 for c in rgb) + (a,)


def frame_markers(frame: Frame, opt: ViewOptions) -> list[dict]:
    """Markers for one frame, as plain dicts: ns, id, type, pos, size, rgba, points, text."""
    out: list[dict] = []
    if opt.intents:
        k = 0
        for b in frame.intents:
            if opt.certified_only and not b.certified:
                continue
            rgb = VEHICLE_PALETTE[b.vid % len(VEHICLE_PALETTE)]
            out.append({"ns": "intents", "id": k, "type": "box", "pos": tuple(b.center), "size": tuple(b.size),
                        "rgba": _rgba(rgb, 0.10 if b.certified else 0.04)})   # faint: up to ~20 overlap
            k += 1
    if opt.truth:
        for k, (vid, p) in enumerate(sorted(frame.truth.items())):
            out.append({"ns": "truth", "id": k, "type": "sphere", "pos": tuple(p), "size": (2.0, 2.0, 2.0),
                        "rgba": _rgba(TRUTH_RGB, 0.7)})
    if opt.predictions:
        pts = frame.predicted.get(opt.prediction_horizon) or {}
        for k, (vid, p) in enumerate(sorted(pts.items())):
            out.append({"ns": "predicted", "id": k, "type": "sphere", "pos": tuple(p), "size": (3.0, 3.0, 3.0),
                        "rgba": _rgba(PRED_RGB, 0.8)})
            cur = frame.vehicles.get(vid)
            if cur is not None:
                out.append({"ns": "predicted_path", "id": k, "type": "line_strip", "points": [tuple(cur.pos), tuple(p)],
                            "rgba": _rgba(PRED_RGB, 0.6)})
    if opt.labels:
        for k, (vid, v) in enumerate(sorted(frame.vehicles.items())):
            out.append({"ns": "labels", "id": k, "type": "text", "pos": tuple(v.pos + np.array([0, 0, 2.5 * opt.scale])),
                        "size": (3.0, 3.0, 3.0), "rgba": _rgba(OK_RGB if v.conforming else NC_RGB),
                        "text": f"{vid}" if v.conforming else f"{vid} !"})
    return out


class GzView:
    MARKER_RETRY_S = 30.0

    def __init__(self, client, opt: ViewOptions = ViewOptions(), clock=time.monotonic):
        self.gz, self.opt, self.clock = client, opt, clock
        self.models: dict[int, float] = {}            # vid -> last time seen
        self.marker_ids: dict[str, int] = {}          # ns -> how many ids were in use last frame
        self.markers_off_until = -math.inf            # no GUI answering /marker_array: skip markers until then
        self.markers_warned = False

    @staticmethod
    def names(vid: int) -> tuple[str, str]:
        return f"veh_{vid}_ok", f"veh_{vid}_nc"

    def _spawn(self, vid: int, pos):
        ok, nc = self.names(vid)
        park = (pos[0], pos[1], PARK_Z)
        self.gz.create(drone_marker_sdf(ok, OK_RGB, self.opt.scale), ok, pos)
        self.gz.create(drone_marker_sdf(nc, NC_RGB, self.opt.scale), nc, park)

    def apply(self, frame: Frame):
        if self.opt.vehicles:
            self._vehicles(frame)
        self._markers(frame_markers(frame, self.opt))

    def _vehicles(self, frame: Frame):
        poses = []
        for vid, v in frame.vehicles.items():
            if vid not in self.models:
                self._spawn(vid, v.pos)
            self.models[vid] = frame.t
            sp = float(np.hypot(v.vel[0], v.vel[1]))
            yaw = math.atan2(v.vel[1], v.vel[0]) if sp > 0.5 else 0.0
            tilt = min(0.35, 0.03 * sp)                         # lean into the motion, for the eye
            q = rpy_to_quat(0.0, tilt, yaw)
            ok, nc = self.names(vid)
            p = tuple(float(x) for x in v.pos)
            park = (p[0], p[1], PARK_Z)
            poses += [(ok, p if v.conforming else park, q), (nc, park if v.conforming else p, q)]
        for vid in [k for k, seen in self.models.items() if k not in frame.vehicles]:
            if frame.t - self.models[vid] > self.opt.forget_s:
                for n in self.names(vid):
                    self.gz.remove(n)
                del self.models[vid]
            else:
                ok, nc = self.names(vid)
                poses += [(ok, (0, 0, PARK_Z), (0, 0, 0, 1)), (nc, (0, 0, PARK_Z), (0, 0, 0, 1))]
        self.gz.set_poses(poses)

    def _markers(self, mk: list[dict]):
        """Markers are drawn by the GUI. Without one, each request waits for its timeout, so after a failure they
        are skipped for MARKER_RETRY_S (and the ids are forgotten: a GUI that appears later starts clean)."""
        now = self.clock()
        if now < self.markers_off_until:
            return
        used: dict[str, int] = {}
        for m in mk:
            used[m["ns"]] = max(used.get(m["ns"], 0), m["id"] + 1)
        for ns, n_old in self.marker_ids.items():
            for i in range(used.get(ns, 0), n_old):
                mk.append({"ns": ns, "id": i, "action": "delete"})
        # Modifying a line marker in Gazebo appends its new points to the old ones (seen in the GUI as fans of
        # lines through every past position), so lines are deleted and drawn again each frame.
        out = []
        for m in mk:
            if m.get("type") in LINE_TYPES and self.marker_ids.get(m["ns"], 0) > m["id"]:
                out.append({"ns": m["ns"], "id": m["id"], "action": "delete"})
            out.append(m)
        mk = out
        if self.gz.markers(mk) is False:
            self.markers_off_until = now + self.MARKER_RETRY_S
            self.marker_ids = {}
            if not self.markers_warned:
                print(f"gz view: no Gazebo GUI is drawing markers (/marker_array); intent boxes are skipped, "
                      f"retrying every {self.MARKER_RETRY_S:.0f} s")
                self.markers_warned = True
            return
        self.marker_ids = used

    def clear(self):
        for vid in list(self.models):
            for n in self.names(vid):
                self.gz.remove(n)
        self.models.clear()
        self._markers([])


class RecordingClient:
    """Stands in for `Gz`: records calls (tests, and `gz_view.py --dump` without Gazebo)."""

    def __init__(self):
        self.calls: list = []
        self.alive: set = set()

    def create(self, sdf, name, pos=(0, 0, 0), quat=(0, 0, 0, 1)):
        self.alive.add(name)
        self.calls.append(("create", name, tuple(float(x) for x in pos)))
        return True

    def remove(self, name):
        self.alive.discard(name)
        self.calls.append(("remove", name))
        return True

    def set_poses(self, poses):
        self.calls.append(("set_poses", [(n, [round(float(x), 2) for x in p]) for n, p, _q in poses]))
        return True

    def markers(self, markers):
        self.calls.append(("markers", markers))
        return True
