"""Export a twin run to a Rerun recording (3D, scrubbable timeline).

    pip install rerun-sdk                     # on the machine with a display (Windows is fine)
    python scripts/twin_view.py runs_twin/mini_town_tier_a_s1_x1     # -> twin.rrd
    rerun runs_twin/mini_town_tier_a_s1_x1/twin.rrd

Shows the OSM world (buildings extruded, no-fly zones, hubs), the twin's vehicle estimates (green when
conforming, red when not), the plant's true positions (grey, sim plants), the intent boxes in force at
each moment, the twin's 60 s position predictions, and alerts / replans / calibration as a text log.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_sim.osm import Airspace  # noqa: E402
from swarm_sim.twin.log import read_log  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("run", type=Path)
ap.add_argument("--world-dir", type=Path, default=ROOT / "data/worlds")
ap.add_argument("--every", type=float, default=1.0, help="seconds between frames")
args = ap.parse_args()

import rerun as rr  # noqa: E402
from swarm_sim.viz.scene import world_mesh  # noqa: E402

rep = json.loads((args.run / "report.json").read_text())
air = Airspace.load(args.world_dir / rep["world"] / "airspace.json")
vers = json.loads((args.run / "intents.json").read_text())["versions"]
logf = args.run / "twin_log.mcap"
if not logf.exists():
    logf = args.run / "twin_log.jsonl"

rr.init(f"twin {args.run.name}", spawn=False)
out = args.run / "twin.rrd"
rr.save(str(out))

# ------------------------------------------------------------------ static world
verts, tris = world_mesh(air)
rr.log("world/buildings", rr.Mesh3D(vertex_positions=verts, triangle_indices=tris,
                                    albedo_factor=[170, 170, 180, 255]), static=True)
for k, nf in enumerate(air.no_fly):
    ring = np.array(nf.polygon + [nf.polygon[0]], float)
    rr.log(f"world/no_fly/{k}", rr.LineStrips3D([np.c_[ring, np.zeros(len(ring))], np.c_[ring, np.full(len(ring), air.ceiling_m)]],
                                                colors=[[220, 60, 60]]), static=True)
rr.log("world/hubs", rr.Points3D([[h.x, h.y, h.elevation + 1] for h in air.hubs], radii=6, colors=[[40, 120, 200]],
                                 labels=[h.name for h in air.hubs]), static=True)

# ------------------------------------------------------------------ dynamic
PAL = np.array([[31, 119, 180], [255, 127, 14], [44, 160, 44], [148, 103, 189], [140, 86, 75], [227, 119, 194],
                [188, 189, 34], [23, 190, 207]])
last = -1e9
for topic, t, msg in read_log(logf):
    rr.set_time("sim", duration=t)
    if topic in ("/twin/alert", "/twin/replan", "/twin/calibration", "/twin/decision"):
        lvl = "WARN" if topic == "/twin/alert" and msg.get("event") == "nonconforming" else "INFO"
        rr.log("events", rr.TextLog(f"{topic.split('/')[-1]}: {json.dumps(msg)}", level=lvl))
        continue
    if topic == "/twin/prediction":
        pts = msg["pos"].get("60.0") or msg["pos"].get("60") or {}
        if pts:
            rr.log("twin/predicted_60s", rr.Points3D(list(pts.values()), radii=2, colors=[[250, 200, 60]]))
        continue
    if topic == "/plant/truth":
        v = msg["vehicles"]
        rr.log("plant/truth", rr.Points3D(list(v.values()) or np.zeros((0, 3)), radii=2.5, colors=[[120, 120, 120]]))
        continue
    if topic != "/twin/state" or t - last < args.every - 1e-6:
        continue
    last = t
    v = msg["vehicles"]
    ids = [int(k) for k in v]
    pos = [v[k]["pos"] for k in v]
    col = [[40, 180, 80] if v[k]["conforming"] else [230, 40, 40] for k in v]
    rr.log("twin/vehicles", rr.Points3D(pos or np.zeros((0, 3)), radii=4, colors=col or None,
                                        labels=[str(i) for i in ids] or None))
    tm = t * 1000
    mins, sizes, cols = [], [], []
    for x in vers:
        if not (x["valid_from"] <= t <= x["valid_to"]):
            continue
        for b in x["int"]:
            if b[6] <= tm <= b[7]:
                mins.append([b[0] / 1000, b[2] / 1000, b[4] / 1000])
                sizes.append([(b[1] - b[0]) / 1000, (b[3] - b[2]) / 1000, (b[5] - b[4]) / 1000])
                cols.append([*PAL[x["vid"] % len(PAL)], 60])
    rr.log("twin/intents", rr.Boxes3D(mins=mins or np.zeros((0, 3)), sizes=sizes or np.zeros((0, 3)),
                                      colors=cols or None))
print(f"-> {out}  (open with: rerun {out})")
