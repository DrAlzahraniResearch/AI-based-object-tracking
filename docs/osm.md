# OSM → world

```bash
# From a saved extract (JOSM export, or the Overpass query below saved as .osm):
python scripts/osm_to_world.py --osm path/to/area.osm --name my_area --hubs configs/hubs_my_area.yaml
# Directly from Overpass (needs network; keep the bbox small, ~1-2 km):
python scripts/osm_to_world.py --fetch S W N E --name my_area
```

Output in `data/worlds/<name>/`: `airspace.json` (used by Tier A), `world.sdf` (Gazebo Jetty),
`preview.png`, `summary.json`. Data under `data/` is not tracked; OSM data is © OpenStreetMap
contributors, ODbL. Credit it in any figure.

## What is extracted

* **Buildings** (ways and multipolygon relations, outer rings). Height from `height`, then
  `building:levels` × 3 m, then a per-type default (house 6 m, commercial 9 m, hospital 18 m, …).
* **No-fly zones**: `aeroway=aerodrome|helipad`, `landuse=military`, `amenity=prison`,
  `power=plant|substation`. These are placeholders for real airspace data. In the US, FAA UAS
  Facility Maps and TFRs are the authority; OSM tags are not.
* **Roads** for the preview and as optional corridors.
* **Hubs** from a YAML file (`x/y` in metres or `lat/lon`), with pad positions. `hub_clearance_problems`
  warns when a hub sits in a building or no-fly zone.

## Projection

Equirectangular about the origin (bbox centre by default). Distortion grows with distance from the
origin; it is negligible for the 1–2 km extracts used here, but don't use one frame for a whole city. The SDF world includes `<spherical_coordinates>` with the same origin, so Gazebo's
NavSat sensor and PX4's GPS agree with the local ENU frame.

## Gazebo geometry

`--building-geometry polyline` (default) extrudes the exact footprint; `box` uses the minimum rotated
rectangle, which is faster to load for large areas. Load the world in the `sim` container:

```bash
docker compose -f docker/compose.yaml run --rm sim gz sim -s -r --iterations 100 data/worlds/mini_town/world.sdf
```
