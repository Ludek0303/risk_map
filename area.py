"""GPS mapping-area polygon: the area a mapping flight is meant to cover,
given top-down (before the flight) rather than derived from where the
vehicle happens to be.

Load it from a JSON file holding a flat list of [lat, lon] vertices, e.g.::

    [[52.2297, 21.0122], [52.2299, 21.0130], [52.2291, 21.0135]]

`mapping_capture.py` uses `GpsPolygon.contains()` against live telemetry to
tell when the vehicle has actually reached the area, so image capture (and
the in-flight map updates) start there rather than the moment it's armed.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import List, Tuple, Union

METERS_PER_DEGREE_LAT = 111320.0


class GpsPolygon:
    """A ground-area boundary, tested in a local flat-earth projection
    (fine at the scale of a single mapping flight)."""

    def __init__(self, vertices: List[Tuple[float, float]]) -> None:
        if len(vertices) < 3:
            raise ValueError("A mapping-area polygon needs at least 3 [lat, lon] vertices.")
        self.vertices = vertices
        self.ref_lat = sum(lat for lat, _ in vertices) / len(vertices)
        self.ref_lon = sum(lon for _, lon in vertices) / len(vertices)
        self._local = [self.to_local_m(lat, lon) for lat, lon in vertices]

    def to_local_m(self, lat: float, lon: float) -> Tuple[float, float]:
        """Project a lat/lon into local (east_m, north_m) offsets from this
        polygon's reference point (its vertex centroid) - the same flat-earth
        projection used internally for contains(), exposed so other modules
        (live_mosaic.py) can place points in the same local frame."""
        north = (lat - self.ref_lat) * METERS_PER_DEGREE_LAT
        east = (lon - self.ref_lon) * METERS_PER_DEGREE_LAT * math.cos(math.radians(self.ref_lat))
        return east, north

    def bounding_box_m(self) -> Tuple[float, float, float, float]:
        """(min_east, min_north, max_east, max_north) of the polygon's
        vertices in the same local meter frame as to_local_m()."""
        east = [e for e, _ in self._local]
        north = [n for _, n in self._local]
        return min(east), min(north), max(east), max(north)

    def contains(self, lat: float, lon: float) -> bool:
        """Ray-casting point-in-polygon test (even-odd rule)."""
        x, y = self.to_local_m(lat, lon)
        inside = False
        n = len(self._local)
        for i in range(n):
            x1, y1 = self._local[i]
            x2, y2 = self._local[(i + 1) % n]
            if (y1 > y) != (y2 > y):
                x_at_y = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                if x < x_at_y:
                    inside = not inside
        return inside

    @classmethod
    def load(cls, path: Union[str, Path]) -> "GpsPolygon":
        data = json.loads(Path(path).read_text())
        vertices = [(float(lat), float(lon)) for lat, lon in data]
        return cls(vertices)
