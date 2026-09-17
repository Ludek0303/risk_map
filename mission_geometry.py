"""Local geometry and JSON input for four-corner survey missions."""
import json
import math
import re
from pathlib import Path


def gps_points(value, name):
    if not isinstance(value, list):
        raise ValueError(f"{name} must be a list of [latitude, longitude] points")
    points = []
    for point in value:
        if (not isinstance(point, list) or len(point) != 2
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in point)):
            raise ValueError(f"{name}: each point must contain two numbers [latitude, longitude]")
        lat, lon = point
        if not (-89 < lat < 89 and -180 <= lon <= 180):
            raise ValueError(f"{name}: invalid GPS coordinates")
        points.append((lat, lon))
    return points


def read_config_json(path):
    """Read JSON with // and /* */ comments, preserving quoted strings."""
    text = Path(path).read_text()
    token = r'"(?:\\.|[^"\\])*"|//[^\r\n]*|/\*[\s\S]*?\*/'
    def remove_comment(match):
        value = match.group()
        if value.startswith('"'):
            return value
        return ''.join('\n' if c == '\n' else ' ' for c in value)
    def unique_object(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"Duplicate configuration field {key!r}; leave only one boundary uncommented")
            result[key] = value
        return result
    return json.loads(re.sub(token, remove_comment, text), object_pairs_hook=unique_object)


FT_TO_M = 0.3048
ALTITUDE_UNIT_PAIRS = (("lap_altitude_m", "lap_altitude_ft"), ("riskmapping_altitude_m", "riskmapping_altitude_ft"))


def load_config(path):
    data = read_config_json(path)
    if not isinstance(data, dict):
        raise ValueError("Mission configuration must be a JSON object")
    setting_names = ({"lap_altitude_m", "riskmapping_altitude_m", "lap_speed_m_s",
                     "riskmapping_speed_m_s", "transfer_speed_m_s", "lap_delay_s", "riskmapping_delay_s"}
                     | {ft_name for _, ft_name in ALTITUDE_UNIT_PAIRS})
    for m_name, ft_name in ALTITUDE_UNIT_PAIRS:
        if m_name in data and ft_name in data:
            raise ValueError(f"Use either {m_name} or {ft_name}, not both")
    unknown = set(data) - ({"area_points", "lap_points", "laps", "flight_boundary", "_description", "search_boundary", "search_boundaries"} | setting_names)
    if unknown:
        raise ValueError(f"Unknown configuration fields: {sorted(unknown)}")
    if "search_boundaries" in data or "search_boundary" in data:
        if "area_points" in data:
            raise ValueError("Use search_boundaries or area_points, not both")
        selected = data.get("search_boundary")
        if type(selected) is not int or selected not in (1, 2):
            raise ValueError("search_boundary must be 1 or 2")
        boundaries = data.get("search_boundaries")
        if not isinstance(boundaries, dict) or set(boundaries) != {"1", "2"}:
            raise ValueError('search_boundaries must contain polygons "1" and "2"')
        areas = {}
        for key, points in boundaries.items():
            areas[key] = gps_points(points, f"search_boundaries.{key}")
            if len(areas[key]) != 4:
                raise ValueError(f"search_boundaries.{key} must contain exactly four corners")
            polygon_grid(areas[key], *areas[key][0], 20.0)
        area = areas[str(selected)]
    else:
        area = gps_points(data.get("area_points"), "area_points")
    if len(area) != 4:
        raise ValueError("area_points must contain exactly four corners")
    lap = gps_points(data.get("lap_points", []), "lap_points")
    laps = data.get("laps", 0)
    if isinstance(laps, bool) or not isinstance(laps, int) or laps < 0:
        raise ValueError("laps must be a non-negative integer")
    if lap and lap[-1] == lap[0]:
        lap.pop()  # Closing point is added automatically.
    if laps and len(set(lap)) < 2:
        raise ValueError("At least two distinct lap_points are required when laps > 0")
    boundary = None
    if "flight_boundary" in data:
        from flight_boundary import FlightBoundary
        boundary = FlightBoundary(data['flight_boundary'])
    settings = {name: data[name] for name in setting_names if name in data}
    for name, value in settings.items():
        is_delay = name in ("lap_delay_s", "riskmapping_delay_s")
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or (value < 0 if is_delay else value <= 0)):
            requirement = "non-negative" if is_delay else "positive"
            raise ValueError(f"{name} must be a {requirement} finite number")
    for m_name, ft_name in ALTITUDE_UNIT_PAIRS:
        if ft_name in settings:
            settings[m_name] = settings.pop(ft_name) * FT_TO_M
    return area, lap, laps, boundary, settings


def to_local(point, lat, lon):
    return ((point[1] - lon) * 111320 * math.cos(math.radians(lat)),
            (point[0] - lat) * 111320)


def polygon_grid(points, lat, lon, spacing):
    """Sort convex corners and clip north/south survey lines to the polygon.

    Coordinates use the existing generator's local flat-earth projection.
    Both endpoints and line centers are inset to permit strict capture gating.
    """
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError("Spacing must be positive and finite")
    local = [to_local(p, lat, lon) for p in points]
    if len(local) != 4 or len(set(local)) != 4:
        raise ValueError("Four distinct area corners are required")
    cx = sum(p[0] for p in local) / 4
    cy = sum(p[1] for p in local) / 4
    order = sorted(range(4), key=lambda i: math.atan2(local[i][1]-cy, local[i][0]-cx))
    polygon = [local[i] for i in order]
    for a, b, c in zip(polygon, polygon[1:]+polygon[:1], polygon[2:]+polygon[:2]):
        cross = (b[0]-a[0])*(c[1]-b[1]) - (b[1]-a[1])*(c[0]-b[0])
        if cross <= 1e-6:
            raise ValueError("Area corners must form a non-degenerate convex quadrilateral")
    if max(math.dist(a, b) for a in polygon for b in polygon) > 20000:
        raise ValueError("Area is too large for the local projection (maximum diagonal 20 km)")
    lo, hi = min(p[0] for p in polygon), max(p[0] for p in polygon)
    inset = min(2., (hi-lo)/4, spacing/4)
    count = max(2, math.ceil((hi-lo-2*inset)/spacing)+1)
    lines = []
    for i in range(count):
        x = lo+inset+i*(hi-lo-2*inset)/(count-1)
        ys = []
        for a, b in zip(polygon, polygon[1:]+polygon[:1]):
            if a[0] != b[0] and min(a[0], b[0]) <= x <= max(a[0], b[0]):
                ys.append(a[1]+(x-a[0])*(b[1]-a[1])/(b[0]-a[0]))
        bottom, top = min(ys), max(ys)
        margin = min(2., (top-bottom)/4, spacing/4)
        line = [(x, bottom+margin), (x, top-margin)]
        lines.append(line if i % 2 == 0 else line[::-1])
    return [points[i] for i in order], lines


def lap_route(points, laps, lat, lon):
    """Approach A, then repeat B ... A so each lap is a complete circuit."""
    if not laps:
        return []
    local = [to_local(p, lat, lon) for p in points]
    return [local[0]] + (local[1:] + local[:1]) * laps
