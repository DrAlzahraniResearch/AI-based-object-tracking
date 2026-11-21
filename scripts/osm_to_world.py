"""Convert an OSM extract into Tier A airspace + a Gazebo world.

    python scripts/osm_to_world.py --osm tests/fixtures/mini_town.osm --hubs configs/hubs_mini_town.yaml
    python scripts/osm_to_world.py --fetch 40.0 -105.01 40.01 -104.99 --name my_area --hubs my_hubs.yaml

Writes data/worlds/<name>/: airspace.json, world.sdf, preview.png, summary.json.
--fetch (south west north east) downloads via the Overpass API (needs internet).
"""
import argparse
import json
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_sim.osm import fetch_overpass, hub_clearance_problems, read_osm, world_sdf  # noqa: E402
from swarm_sim.plot import plot_airspace  # noqa: E402

ap = argparse.ArgumentParser()
src = ap.add_mutually_exclusive_group(required=True)
src.add_argument("--osm", type=Path)
src.add_argument("--fetch", nargs=4, type=float, metavar=("S", "W", "N", "E"))
ap.add_argument("--name")
ap.add_argument("--hubs", type=Path, help="YAML with a 'hubs' list (x/y metres or lat/lon)")
ap.add_argument("--origin", nargs=2, type=float, metavar=("LAT", "LON"), help="default: bbox centre")
ap.add_argument("--building-geometry", choices=["polyline", "box"], default="polyline",
                help="polyline = exact extruded footprints; box = oriented bounding boxes (fallback)")
ap.add_argument("--ceiling", type=float, default=120.0, help="operating ceiling, m above ground")
ap.add_argument("--out", type=Path, default=ROOT / "data/worlds")
args = ap.parse_args()

osm = args.osm
name = args.name or (osm.stem if osm else "area")
out = args.out / name
out.mkdir(parents=True, exist_ok=True)
if args.fetch:
    osm = fetch_overpass(tuple(args.fetch), out / f"{name}.osm")
hubs = yaml.safe_load(args.hubs.read_text())["hubs"] if args.hubs else []
air = read_osm(osm, tuple(args.origin) if args.origin else None, name, hubs, args.ceiling)
air.save(out / "airspace.json")
(out / "world.sdf").write_text(world_sdf(air, args.building_geometry))
plot_airspace(air, out / "preview.png")
probs = hub_clearance_problems(air)
src_counts = {}
for b in air.buildings:
    src_counts[b.height_source] = src_counts.get(b.height_source, 0) + 1
summary = {"name": name, "origin": air.origin, "bounds_m": air.bounds, "buildings": len(air.buildings),
           "building_height_sources": src_counts,
           "max_building_m": max((b.height for b in air.buildings), default=0),
           "no_fly_zones": len(air.no_fly), "roads": len(air.roads), "hubs": [h.name for h in air.hubs],
           "hub_problems": probs}
(out / "summary.json").write_text(json.dumps(summary, indent=1))
print(json.dumps(summary, indent=1))
if probs:
    print("WARNING: move these hubs (pads too close to buildings / inside no-fly):", *probs, sep="\n  ")
