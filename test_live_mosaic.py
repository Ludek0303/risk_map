import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from area import GpsPolygon
from live_mosaic import LiveMosaic


class MosaicTests(unittest.TestCase):
    def setUp(self):
        self.area = GpsPolygon([(-0.001, -0.001), (-0.001, 0.001),
                                (0.001, 0.001), (0.001, -0.001)])
        self.mosaic = LiveMosaic(self.area, 90, pixels_per_meter=1)

    def test_overlapping_views_do_not_average_details(self):
        white = np.full((40, 40, 3), 240, np.uint8)
        dark = np.full_like(white, 20)
        self.mosaic.update(white, 0, 0, 20, 0)
        self.mosaic.update(dark, 0, 10 / 111320, 20, 0)
        values = self.mosaic.canvas[self.mosaic._weight > 0, 0]
        self.assertTrue(np.all((values == 240) | (values == 20)))
        row = round(self.mosaic._max_n)
        col = round(-self.mosaic._min_e)
        self.assertEqual(self.mosaic.canvas[row, col, 0], 240)
        self.assertEqual(self.mosaic.canvas[row, col + 10, 0], 20)

    def test_outside_and_invalid_frames(self):
        frame = np.full((40, 40, 3), 100, np.uint8)
        self.mosaic.update(frame, 1, 1, 20, 90)
        self.assertFalse(self.mosaic.canvas.any())
        with self.assertRaises(ValueError):
            self.mosaic.update(frame, 0, 0, 0, 0)

    def test_rotated_edges_and_png_roundtrip(self):
        frame = np.full((40, 40, 3), 100, np.uint8)
        self.mosaic.update(frame, 0, 0, 20, 37)
        self.assertTrue(np.all(self.mosaic.canvas[self.mosaic._weight > 0] == 100))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "map.png"
            self.mosaic.save(path)
            np.testing.assert_array_equal(cv2.imread(str(path)), self.mosaic.canvas.astype(np.uint8))

    def test_global_alignment_recovers_shifted_views(self):
        from align_mosaic import align_frames
        rng = np.random.default_rng(4)
        texture = rng.integers(0, 256, (320, 480), dtype=np.uint8)
        texture = cv2.GaussianBlur(texture, (3, 3), 0.6)
        records = []
        with tempfile.TemporaryDirectory() as tmp:
            for i, (x, y) in enumerate(((0, 0), (40, 0), (80, 0), (0, 40), (40, 40), (80, 40))):
                path = Path(tmp) / f"{i}.png"
                cv2.imwrite(str(path), texture[y:y+240, x:x+240])
                east, north = x / 6, -y / 6
                records.append((path, north / 111320, east / 111320, 20, 5))
            poses, _, usable = align_frames(records, self.area, 90, 1)
        self.assertTrue(usable.all())
        # GPS anchors recover the known 6 px/m survey despite a 5-degree
        # erroneous heading prior. Test geometry, not just successful output.
        np.testing.assert_allclose(poses[:, 0], 20, atol=0.5)
        np.testing.assert_allclose(poses[:, 1], 0, atol=0.5)
        np.testing.assert_allclose(poses[2, 2] - poses[0, 2], 80 / 6, atol=0.5)


if __name__ == "__main__":
    unittest.main()
