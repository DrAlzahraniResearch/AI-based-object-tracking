"""Export a twin run for Foxglove: one MCAP with a 3D scene, an event log and plottable metrics.

    python scripts/foxglove_export.py runs_twin/<run>            # -> runs_twin/<run>/foxglove.mcap
    # open it in Foxglove (app.foxglove.dev or the desktop app): 3D panel on /world + /scene (frame "world"),
    # Log panel on /events, Plot panel on /metrics.flying, /metrics.nonconforming, /metrics.intents_certified

Pure Python (mcap), runs in the eval image. See docs/visualization.md for which viewer to use when.
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_sim.viz.foxglove import export  # noqa: E402
from swarm_sim.viz.scene import Scene  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("run", type=Path)
ap.add_argument("--world-dir", type=Path, default=ROOT / "data/worlds")
ap.add_argument("--every", type=float, default=1.0, help="sim seconds between scene frames")
ap.add_argument("--start", type=float)
ap.add_argument("--end", type=float)
ap.add_argument("--out", type=Path)
args = ap.parse_args()

scene = Scene.from_run(args.run, args.world_dir)
out = args.out or args.run / "foxglove.mcap"
counts = export(scene, out, args.every, args.start, args.end)
print(json.dumps(counts))
print(f"-> {out}  ({out.stat().st_size / 1e6:.1f} MB)")
