"""Incremental GPS mosaic using the most central available view per pixel.

Frames are projected using position, height above the ground and camera heading.
Selecting a single source preserves detail when GPS placement is imperfect;
averaging misregistered views would produce ghosting. Source changes can still
show seams, and terrain relief / attitude errors require proper photogrammetry.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Union

import cv2
import numpy as np

from area import GpsPolygon


class LiveMosaic:
    def __init__(
        self,
        area: GpsPolygon,
        camera_hfov_deg: float,
        cropped_width_fraction: float = 1.0,
        pixels_per_meter: float = 8.0,
        margin_m: float = 10.0,
    ) -> None:
        """area: the GpsPolygon defining the mapping area - also fixes the
        canvas's local coordinate frame (see area.to_local_m()).
        camera_hfov_deg: the camera's full-sensor horizontal FOV.
        cropped_width_fraction: if frames passed to update() have already
        been cropped narrower than the full sensor (e.g.
        mapping_capture.py's airframe-edge crop), the fraction of the
        original width that remains - needed to get ground-sample-distance
        right, since HFOV describes the *uncropped* sensor.
        """
        if not (0 < camera_hfov_deg < 180 and 0 < cropped_width_fraction <= 1
                and math.isfinite(pixels_per_meter) and pixels_per_meter > 0
                and math.isfinite(margin_m) and margin_m >= 0):
            raise ValueError("Invalid camera geometry, map resolution or margin.")
        self._area = area
        self._hfov_deg = camera_hfov_deg
        self._cropped_width_fraction = cropped_width_fraction
        self._px_per_m = pixels_per_meter

        min_e, min_n, max_e, max_n = area.bounding_box_m()
        self._min_e = min_e - margin_m
        self._max_n = max_n + margin_m
        self._canvas_w = max(1, int(math.ceil((max_e - min_e + 2 * margin_m) * pixels_per_meter)))
        self._canvas_h = max(1, int(math.ceil((max_n - min_n + 2 * margin_m) * pixels_per_meter)))
        self.canvas = np.zeros((self._canvas_h, self._canvas_w, 3), dtype=np.float32)
        self._weight = np.zeros((self._canvas_h, self._canvas_w), dtype=np.float32)

    def update(self, frame: np.ndarray, lat: float, lon: float, alt_m: float, heading_deg: float) -> None:
        """Use a frame where its viewing angle is better than the current source."""
        if not all(math.isfinite(v) for v in (lat, lon, alt_m, heading_deg)) or alt_m <= 0:
            raise ValueError("Position and heading must be finite; ground-relative height must be positive.")
        if frame.ndim != 3 or frame.shape[2] != 3 or min(frame.shape[:2]) < 2:
            raise ValueError("Expected a nonempty BGR image of at least 2x2 pixels.")
        h, w = frame.shape[:2]

        footprint_w_full_m = 2 * alt_m * math.tan(math.radians(self._hfov_deg) / 2)
        full_width_px = w / self._cropped_width_fraction
        meters_per_px = footprint_w_full_m / full_width_px
        scale = meters_per_px * self._px_per_m

        east_c, north_c = self._area.to_local_m(lat, lon)
        col_c = (east_c - self._min_e) * self._px_per_m
        row_c = (self._max_n - north_c) * self._px_per_m

        # Vehicle heading (compass degrees, clockwise from north) rotates
        # the image's "up" (vehicle-forward) edge to point at true north.
        theta = math.radians(heading_deg)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        cx, cy = w / 2.0, h / 2.0

        m = np.array([
            [scale * cos_t, -scale * sin_t, col_c - scale * cos_t * cx + scale * sin_t * cy],
            [scale * sin_t, scale * cos_t, row_c - scale * sin_t * cx - scale * cos_t * cy],
        ], dtype=np.float64)

        self.update_projected(frame, m)

    def update_projected(self, frame: np.ndarray, matrix: np.ndarray) -> None:
        """Composite a frame with a precomputed source-to-canvas transform."""
        h, w = frame.shape[:2]
        cx, cy = w / 2.0, h / 2.0
        full_width_px = w / self._cropped_width_fraction
        m = np.asarray(matrix, dtype=np.float64).copy()

        # Work only on the projected footprint, including interpolation support.
        corners = np.array([[0, 0, 1], [w, 0, 1], [0, h, 1], [w, h, 1]]) @ m.T
        x0 = max(0, int(np.floor(corners[:, 0].min())) - 1)
        y0 = max(0, int(np.floor(corners[:, 1].min())) - 1)
        x1 = min(self._canvas_w, int(np.ceil(corners[:, 0].max())) + 1)
        y1 = min(self._canvas_h, int(np.ceil(corners[:, 1].max())) + 1)
        if x1 <= x0 or y1 <= y0:
            return
        m[:, 2] -= (x0, y0)
        size = (x1 - x0, y1 - y0)

        # Continuous centrality (no flat plateau), using equal angular scales
        # on both axes. Explicit zero borders exclude partially sampled edges.
        yy, xx = np.mgrid[:h, :w].astype(np.float32)
        radius2 = ((xx - cx) ** 2 + (yy - cy) ** 2) / (full_width_px / 2) ** 2
        src_weight = 1.0 / (1.0 + radius2)
        src_weight[[0, -1], :] = 0
        src_weight[:, [0, -1]] = 0
        warped = cv2.warpAffine(frame, m, size, flags=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_REPLICATE)
        weight = cv2.warpAffine(src_weight, m, size, flags=cv2.INTER_LINEAR)
        valid = cv2.warpAffine(np.ones((h, w), np.float32), m, size,
                              flags=cv2.INTER_LINEAR) >= 0.999
        old_weight = self._weight[y0:y1, x0:x1]
        replace = valid & (weight > old_weight + 1e-6)
        self.canvas[y0:y1, x0:x1][replace] = warped[replace]
        old_weight[replace] = weight[replace]

    def save(self, path: Union[str, Path]) -> None:
        path = Path(path)
        params = [cv2.IMWRITE_JPEG_QUALITY, 98] if path.suffix.lower() in (".jpg", ".jpeg") else []
        if not cv2.imwrite(str(path), np.clip(self.canvas, 0, 255).astype(np.uint8), params):
            raise OSError(f"Could not write mosaic: {path}")
