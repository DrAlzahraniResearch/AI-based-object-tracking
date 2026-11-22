"""OpenStreetMap -> civilian airspace (Tier A) and Gazebo world (Tier B).

Input: a `.osm` XML extract (openstreetmap.org "Export", JOSM, or an Overpass
query; see docs/osm.md). Output, in a local East-North-Up frame in metres
centred on a chosen origin:

  Airspace      buildings (footprint polygon + base/top height), no-fly zones,
                roads (polylines), landing hubs, bounds. Used by Tier A for
                routing and collision checks, and saved as airspace.json.
  world.sdf     Gazebo (Jetty) world: buildings as extruded polylines (or
                oriented boxes), roads as thin strips, hub pads, translucent
                no-fly columns.

Projection: local tangent plane (equirectangular about the origin). For areas
of a few km the error is centimetres, far below OSM's own accuracy.
"""
from __future__ import annotations

import json
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from pathlib import Path
from xml.sax.saxutils import escape

import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree
from shapely.validation import make_valid

EARTH_R = 6371008.8
LEVEL_HEIGHT_M = 3.0
DEFAULT_HEIGHT_M = {"house": 6.0, "residential": 7.0, "detached": 6.0, "garage": 3.0, "shed": 3.0,
                    "apartments": 12.0, "commercial": 9.0, "retail": 6.0, "industrial": 9.0,
                    "warehouse": 9.0, "school": 9.0, "university": 12.0, "hospital": 18.0,
                    "church": 15.0, "office": 15.0, "yes": 8.0}
NO_FLY_TAGS = {("aeroway", "aerodrome"), ("aeroway", "helipad"), ("aeroway", "heliport"),
               ("landuse", "military"), ("military", "*"), ("amenity", "prison"),
               ("power", "plant"), ("power", "substation")}
ROAD_CLASSES = {"motorway", "trunk", "primary", "secondary", "tertiary", "residential",
                "unclassified", "service", "living_street"}


# ------------------------------------------------------------------ projection
@dataclass(frozen=True)
class LocalFrame:
    lat0: float
    lon0: float

    def to_xy(self, lat, lon):
        lat, lon = np.asarray(lat, float), np.asarray(lon, float)
        x = EARTH_R * math.cos(math.radians(self.lat0)) * np.radians(lon - self.lon0)
        y = EARTH_R * np.radians(lat - self.lat0)
        return x, y

    def to_latlon(self, x, y):
        lat = self.lat0 + np.degrees(np.asarray(y, float) / EARTH_R)
        lon = self.lon0 + np.degrees(np.asarray(x, float) / (EARTH_R * math.cos(math.radians(self.lat0))))
        return lat, lon


# ------------------------------------------------------------------ data model
@dataclass
class Building:
    osm_id: int
    polygon: list            # [[x, y], ...] closed ring, CCW, metres
    height: float            # top above ground, metres
    min_height: float = 0.0
    kind: str = "yes"
    height_source: str = "default"   # tag | levels | default


@dataclass
class NoFly:
    osm_id: int
    polygon: list
    reason: str
    floor: float = 0.0
    ceiling: float = 1e9


@dataclass
class Road:
    osm_id: int
    line: list               # [[x, y], ...]
    kind: str


@dataclass
class Hub:
    name: str
    x: float
    y: float
    pads: int = 4
    pad_spacing_m: float = 8.0
    elevation: float = 0.0   # pad height (0 = ground; set to a roof height for rooftop hubs)

    def pad_positions(self):
        n = max(1, self.pads)
        cols = math.ceil(math.sqrt(n))
        out = []
        for k in range(n):
            r, c = divmod(k, cols)
            out.append((self.x + (c - (cols - 1) / 2) * self.pad_spacing_m,
                        self.y + (r - (math.ceil(n / cols) - 1) / 2) * self.pad_spacing_m))
        return out


@dataclass
class Airspace:
    name: str
    origin: dict             # {"lat": , "lon": }
    bounds: list             # [xmin, ymin, xmax, ymax] metres
    buildings: list = field(default_factory=list)
    no_fly: list = field(default_factory=list)
    roads: list = field(default_factory=list)
    hubs: list = field(default_factory=list)
    ceiling_m: float = 120.0  # operating ceiling (typical small-UAS limit, ~400 ft)

    # -------------------------------------------------------------- queries
    def _index(self):
        if getattr(self, "_tree", None) is None:
            self._polys = [Polygon(b.polygon) for b in self.buildings]
            self._tree = STRtree(self._polys) if self._polys else None
            self._nf_polys = [Polygon(n.polygon) for n in self.no_fly]
        return self._tree

    def building_top_along(self, a, b, radius: float = 0.0) -> float:
        """Highest building top within `radius` of the 2D segment a->b (0 if none)."""
        tree = self._index()
        if tree is None:
            return 0.0
        geom = LineString([a, b]) if (a[0], a[1]) != (b[0], b[1]) else Point(a)
        if radius > 0:
            geom = geom.buffer(radius)
        idx = tree.query(geom, predicate="intersects")
        return max((self.buildings[i].height for i in idx), default=0.0)

    def inside_building(self, x, y, z) -> bool:
        tree = self._index()
        if tree is None:
            return False
        p = Point(x, y)
        for i in tree.query(p, predicate="intersects"):
            b = self.buildings[i]
            if b.min_height <= z <= b.height:
                return True
        return False

    def in_no_fly(self, x, y, z) -> bool:
        self._index()
        p = Point(x, y)
        return any(poly.contains(p) and n.floor <= z <= n.ceiling for poly, n in zip(self._nf_polys, self.no_fly))

    # -------------------------------------------------------------- io
    def to_json(self) -> dict:
        return {"name": self.name, "origin": self.origin, "bounds": self.bounds, "ceiling_m": self.ceiling_m,
                "buildings": [asdict(b) for b in self.buildings], "no_fly": [asdict(n) for n in self.no_fly],
                "roads": [asdict(r) for r in self.roads], "hubs": [asdict(h) for h in self.hubs]}

    def save(self, path: Path):
        Path(path).write_text(json.dumps(self.to_json()))

    @classmethod
    def load(cls, path: Path) -> "Airspace":
        d = json.loads(Path(path).read_text())
        return cls(d["name"], d["origin"], d["bounds"], [Building(**b) for b in d["buildings"]],
                   [NoFly(**n) for n in d["no_fly"]], [Road(**r) for r in d["roads"]],
                   [Hub(**h) for h in d["hubs"]], d.get("ceiling_m", 120.0))


# ------------------------------------------------------------------ parsing
_NUM = re.compile(r"[-+]?\d+(?:\.\d+)?")


def parse_length_m(s: str | None) -> float | None:
    """'12', '12 m', '12.5m', "40'", '40 ft' -> metres. None if unparseable."""
    if not s:
        return None
    m = _NUM.search(s)
    if not m:
        return None
    v = float(m.group())
    if "'" in s or "ft" in s.lower():
        v *= 0.3048
    return v if v >= 0 else None


def building_height(tags: dict) -> tuple[float, float, str]:
    """(top, base, source) from OSM tags: height > building:levels > per-type default."""
    base = parse_length_m(tags.get("min_height"))
    if base is None and tags.get("building:min_level"):
        lv = parse_length_m(tags["building:min_level"])
        base = lv * LEVEL_HEIGHT_M if lv is not None else None
    base = base or 0.0
    h = parse_length_m(tags.get("height"))
    if h is not None and h > base:
        return h, base, "tag"
    lv = parse_length_m(tags.get("building:levels"))
    if lv is not None and lv > 0:
        roof = parse_length_m(tags.get("roof:levels")) or 0.0
        return base + (lv + 0.5 * roof) * LEVEL_HEIGHT_M, base, "levels"
    kind = tags.get("building", "yes")
    return base + DEFAULT_HEIGHT_M.get(kind, DEFAULT_HEIGHT_M["yes"]), base, "default"


def _is_no_fly(tags: dict) -> str | None:
    for k, v in NO_FLY_TAGS:
        if k in tags and (v == "*" or tags[k] == v):
            return f"{k}={tags[k]}"
    return None


def _ring(coords) -> list | None:
    poly = make_valid(Polygon(coords))
    if poly.geom_type == "MultiPolygon":
        poly = max(poly.geoms, key=lambda g: g.area)
    if poly.geom_type != "Polygon" or poly.area < 1.0:
        return None
    poly = poly.simplify(0.2, preserve_topology=True)
    ext = list(poly.exterior.coords)
    if Polygon(ext).exterior.is_ccw is False:
        ext = ext[::-1]
    return [[round(x, 3), round(y, 3)] for x, y in ext]


def read_osm(path: Path, origin: tuple[float, float] | None = None, name: str | None = None,
             hubs: list[dict] | None = None, ceiling_m: float = 120.0) -> Airspace:
    """Parse an .osm XML file into an Airspace in a local metric frame."""
    root = ET.parse(path).getroot()
    nodes = {}
    for n in root.iter("node"):
        nodes[int(n.get("id"))] = (float(n.get("lat")), float(n.get("lon")))
    ways = {}
    for w in root.iter("way"):
        refs = [int(nd.get("ref")) for nd in w.iter("nd")]
        tags = {t.get("k"): t.get("v") for t in w.iter("tag")}
        ways[int(w.get("id"))] = (refs, tags)
    b = root.find("bounds")
    if origin is None:
        if b is not None:
            origin = ((float(b.get("minlat")) + float(b.get("maxlat"))) / 2,
                      (float(b.get("minlon")) + float(b.get("maxlon"))) / 2)
        else:
            lats, lons = zip(*nodes.values())
            origin = (float(np.mean(lats)), float(np.mean(lons)))
    frame = LocalFrame(*origin)

    def xy(refs):
        pts = [nodes[r] for r in refs if r in nodes]
        if not pts:
            return []
        lat, lon = zip(*pts)
        x, y = frame.to_xy(lat, lon)
        return list(zip(x.tolist(), y.tolist()))

    air = Airspace(name or Path(path).stem, {"lat": origin[0], "lon": origin[1]}, [0, 0, 0, 0],
                   ceiling_m=ceiling_m)
    for wid, (refs, tags) in ways.items():
        closed = len(refs) >= 4 and refs[0] == refs[-1]
        if "building" in tags and closed:
            ring = _ring(xy(refs))
            if ring:
                top, base, src = building_height(tags)
                air.buildings.append(Building(wid, ring, round(top, 2), round(base, 2), tags["building"], src))
        elif closed and (reason := _is_no_fly(tags)):
            ring = _ring(xy(refs))
            if ring:
                air.no_fly.append(NoFly(wid, ring, reason))
        elif tags.get("highway") in ROAD_CLASSES and len(refs) >= 2:
            pts = xy(refs)
            if len(pts) >= 2:
                air.roads.append(Road(wid, [[round(x, 2), round(y, 2)] for x, y in pts], tags["highway"]))
    # multipolygon relations: outer ways only (inner courtyards are ignored -> conservative)
    for rel in root.iter("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.iter("tag")}
        if tags.get("type") != "multipolygon" or not ("building" in tags or _is_no_fly(tags)):
            continue
        for m in rel.iter("member"):
            if m.get("type") == "way" and m.get("role") == "outer" and int(m.get("ref")) in ways:
                refs, _ = ways[int(m.get("ref"))]
                ring = _ring(xy(refs)) if len(refs) >= 4 and refs[0] == refs[-1] else None
                if not ring:
                    continue
                if "building" in tags:
                    top, base, src = building_height(tags)
                    air.buildings.append(Building(int(rel.get("id")), ring, round(top, 2), round(base, 2),
                                                  tags["building"], src))
                else:
                    air.no_fly.append(NoFly(int(rel.get("id")), ring, _is_no_fly(tags)))
    if b is not None:
        x0, y0 = frame.to_xy(float(b.get("minlat")), float(b.get("minlon")))
        x1, y1 = frame.to_xy(float(b.get("maxlat")), float(b.get("maxlon")))
        air.bounds = [round(float(x0), 2), round(float(y0), 2), round(float(x1), 2), round(float(y1), 2)]
    else:
        allxy = np.array([p for bd in air.buildings for p in bd.polygon] or [[0, 0]])
        air.bounds = [*np.round(allxy.min(0), 2).tolist(), *np.round(allxy.max(0), 2).tolist()]
    for h in hubs or []:
        if "lat" in h:
            hx, hy = frame.to_xy(h["lat"], h["lon"])
            hx, hy = float(hx), float(hy)
        else:
            hx, hy = float(h["x"]), float(h["y"])
        air.hubs.append(Hub(h["name"], round(hx, 2), round(hy, 2), int(h.get("pads", 4)),
                            float(h.get("pad_spacing_m", 8.0)), float(h.get("elevation", 0.0))))
    return air


def hub_clearance_problems(air: Airspace, radius_m: float = 15.0) -> list[str]:
    """Hubs whose pads sit inside or right next to a building or no-fly zone."""
    probs = []
    for h in air.hubs:
        for k, (px, py) in enumerate(h.pad_positions()):
            if air.building_top_along((px, py), (px, py), radius_m) > h.elevation + 0.5:
                probs.append(f"hub {h.name} pad {k}: building within {radius_m} m")
            if air.in_no_fly(px, py, h.elevation + 1):
                probs.append(f"hub {h.name} pad {k}: inside a no-fly zone")
    return probs


# ------------------------------------------------------------------ Gazebo world
def _pose(x, y, z, yaw=0.0):
    return f"{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.6f}"


def _material(rgb, alpha=1.0):
    r, g, b = rgb
    tr = "" if alpha >= 1 else f"<transparency>{1 - alpha:.2f}</transparency>"
    return (f"<material><ambient>{r} {g} {b} {alpha}</ambient><diffuse>{r} {g} {b} {alpha}</diffuse>"
            f"<specular>0.05 0.05 0.05 1</specular></material>{tr}")


def _building_visual(b: Building, geometry: str) -> str:
    h = max(0.5, b.height - b.min_height)
    if geometry == "box":
        rect = Polygon(b.polygon).minimum_rotated_rectangle
        c = list(rect.exterior.coords)[:4]
        e0 = np.subtract(c[1], c[0]); e1 = np.subtract(c[2], c[1])
        cx, cy = rect.centroid.x, rect.centroid.y
        yaw = math.atan2(e0[1], e0[0])
        geom = f"<box><size>{np.linalg.norm(e0):.3f} {np.linalg.norm(e1):.3f} {h:.3f}</size></box>"
        pose = _pose(cx, cy, b.min_height + h / 2, yaw)
    else:  # extruded polyline, model frame at the footprint's base
        pts = "".join(f"<point>{x:.3f} {y:.3f}</point>" for x, y in b.polygon[:-1])
        geom = f"<polyline>{pts}<height>{h:.3f}</height></polyline>"
        pose = _pose(0, 0, b.min_height)
    shade = 0.55 + 0.25 * ((b.osm_id * 2654435761) % 1000) / 1000.0   # slight per-building variation
    return f"""
    <model name="bldg_{b.osm_id}"><static>true</static><pose>{pose}</pose>
      <link name="link">
        <visual name="v"><geometry>{geom}</geometry>{_material((shade, shade * 0.97, shade * 0.92))}</visual>
        <collision name="c"><geometry>{geom}</geometry></collision>
      </link>
    </model>"""


# The world systems PX4 v1.16's Gazebo (Harmonic) bridge expects: its server.config minus the two PX4 plugins
# (optical flow, GStreamer camera) that the x500 does not use. A world that lists any system replaces the
# server config, so all of them are listed here.
PX4_GZ_SYSTEMS = ("Physics", "UserCommands", "SceneBroadcaster", "Contact", "Imu", "AirPressure", "AirSpeed",
                  "ApplyLinkWrench", "NavSat", "Magnetometer", "Sensors")
VIEW_GZ_SYSTEMS = ("Physics", "UserCommands", "SceneBroadcaster", "Sensors", "Imu", "NavSat")


def _gz_system(name: str) -> str:
    lib = {"NavSat": "navsat"}.get(name) or re.sub(r"(?<!^)(?=[A-Z])", "-", name).lower()
    body = "<render_engine>ogre2</render_engine>" if name == "Sensors" else ""
    return (f'    <plugin filename="gz-sim-{lib}-system" name="gz::sim::systems::{name}">{body}</plugin>\n'
            if body else f'    <plugin filename="gz-sim-{lib}-system" name="gz::sim::systems::{name}"/>\n')


def world_sdf(air: Airspace, building_geometry: str = "polyline", road_width_m: float = 6.0,
              max_roads: int = 2000, px4: bool = False, real_time_factor: float = 1.0) -> str:
    """Gazebo world. Buildings carry collision geometry so PX4 vehicles can actually hit them.

    px4=False: the viewing world (Jetty or Harmonic): rendering, sensors, kinematic markers (scripts/gz_view.py).
    px4=True: the Tier B world for PX4 SITL in Gazebo Harmonic (swarm_sim/twin/px4_gz.py): PX4's full system
    list, ODE at 250 Hz (PX4's lockstep rate), the magnetic field and atmosphere its sensors read."""
    xmin, ymin, xmax, ymax = air.bounds
    size = max(xmax - xmin, ymax - ymin) + 400
    cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
    parts = []
    for b in air.buildings:
        parts.append(_building_visual(b, building_geometry))
    n_seg = 0
    road_vis = []
    for r in air.roads:
        for (x0, y0), (x1, y1) in zip(r.line[:-1], r.line[1:]):
            L = math.hypot(x1 - x0, y1 - y0)
            if L < 0.5 or n_seg >= max_roads:
                continue
            n_seg += 1
            road_vis.append(f"""
        <visual name="r{n_seg}"><pose>{_pose((x0 + x1) / 2, (y0 + y1) / 2, 0.02, math.atan2(y1 - y0, x1 - x0))}</pose>
          <geometry><box><size>{L + road_width_m * 0.5:.3f} {road_width_m} 0.02</size></box></geometry>{_material((0.25, 0.25, 0.27))}</visual>""")
    if road_vis:
        parts.append(f"""
    <model name="roads"><static>true</static><link name="link">{''.join(road_vis)}
      </link></model>""")
    for h in air.hubs:
        pads = "".join(f"""
        <visual name="pad{k}"><pose>{_pose(px - h.x, py - h.y, 0.03)}</pose>
          <geometry><box><size>4 4 0.05</size></box></geometry>{_material((0.95, 0.75, 0.1))}</visual>"""
                       for k, (px, py) in enumerate(h.pad_positions()))
        parts.append(f"""
    <model name="hub_{escape(h.name)}"><static>true</static><pose>{_pose(h.x, h.y, h.elevation)}</pose>
      <link name="link">{pads}
      </link></model>""")
    for n in air.no_fly:
        pts = "".join(f"<point>{x:.3f} {y:.3f}</point>" for x, y in n.polygon[:-1])
        ceil = min(n.ceiling, air.ceiling_m)
        parts.append(f"""
    <model name="nofly_{n.osm_id}"><static>true</static><pose>{_pose(0, 0, n.floor)}</pose>
      <link name="link"><visual name="v"><geometry><polyline>{pts}<height>{ceil - n.floor:.1f}</height></polyline></geometry>
        {_material((0.9, 0.1, 0.1), alpha=0.15)}<cast_shadows>false</cast_shadows></visual></link>
    </model>""")
    if px4:
        physics = (f'    <physics name="4ms" type="ode"><max_step_size>0.004</max_step_size>'
                   f'<real_time_factor>{real_time_factor:g}</real_time_factor>'
                   f'<real_time_update_rate>{250 * real_time_factor:g}</real_time_update_rate></physics>\n'
                   '    <gravity>0 0 -9.8</gravity>\n    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>\n'
                   '    <atmosphere type="adiabatic"/>\n')
    else:
        physics = (f'    <physics name="4ms" type="ignored"><max_step_size>0.004</max_step_size>'
                   f'<real_time_factor>{real_time_factor:g}</real_time_factor></physics>\n')
    systems = "".join(_gz_system(n) for n in (PX4_GZ_SYSTEMS if px4 else VIEW_GZ_SYSTEMS))
    return f"""<?xml version="1.0"?>
<!-- Generated by swarm_sim.osm from {escape(air.name)} (origin {air.origin['lat']:.6f}, {air.origin['lon']:.6f}). Do not edit. -->
<sdf version="1.9">
  <world name="{escape(air.name)}">
{physics}{systems}    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{air.origin['lat']:.8f}</latitude_deg><longitude_deg>{air.origin['lon']:.8f}</longitude_deg>
      <elevation>0</elevation><heading_deg>0</heading_deg>
    </spherical_coordinates>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.7 0.8 0.9 1</background><shadows>true</shadows><grid>false</grid></scene>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 200 0 0 0</pose>
      <diffuse>0.85 0.85 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.25 -0.9</direction></light>
    <model name="ground"><static>true</static><pose>{_pose(cx, cy, 0)}</pose>
      <link name="link">
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>{size:.1f} {size:.1f}</size></plane></geometry>{_material((0.42, 0.5, 0.35))}</visual>
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>{size:.1f} {size:.1f}</size></plane></geometry></collision>
      </link>
    </model>{''.join(parts)}
  </world>
</sdf>
"""


OVERPASS_QUERY = """[out:xml][timeout:120];
(
  way["building"]({s},{w},{n},{e});
  relation["building"]["type"="multipolygon"]({s},{w},{n},{e});
  way["highway"]({s},{w},{n},{e});
  way["aeroway"]({s},{w},{n},{e});
  way["landuse"="military"]({s},{w},{n},{e});
  way["amenity"="prison"]({s},{w},{n},{e});
  way["power"~"plant|substation"]({s},{w},{n},{e});
);
(._;>;);
out body;"""


def fetch_overpass(bbox: tuple[float, float, float, float], out: Path,
                   url: str = "https://overpass-api.de/api/interpreter") -> Path:
    """Download an extract for (south, west, north, east). Keep areas small (a few km²)."""
    import requests
    s, w, n, e = bbox
    q = OVERPASS_QUERY.format(s=s, w=w, n=n, e=e)
    r = requests.post(url, data={"data": q}, timeout=180)
    r.raise_for_status()
    xml = r.text
    if "<bounds" not in xml:   # Overpass omits <bounds>; add it so the origin is the bbox centre
        i = xml.find("<osm")
        j = xml.find(">", i)
        if i < 0 or j < 0:
            raise ValueError("Overpass reply is not OSM XML")
        xml = xml[:j + 1] + f'\n  <bounds minlat="{s}" minlon="{w}" maxlat="{n}" maxlon="{e}"/>' + xml[j + 1:]
    Path(out).write_text(xml)
    return Path(out)
