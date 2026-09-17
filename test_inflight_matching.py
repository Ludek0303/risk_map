import json
import sqlite3
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from area import GpsPolygon
from inflight_matching import IncrementalCache, InflightMatcher
from orthomosaic import match_session


class InflightTests(unittest.TestCase):
    def make_photos(self, root, count=6):
        rng = np.random.default_rng(123)
        base = rng.integers(0, 256, (420, 600, 3), dtype=np.uint8)
        base = cv2.GaussianBlur(base, (3, 3), .5)
        records = []
        for i in range(count):
            frame = cv2.warpAffine(base, np.float32([[1, 0, i*3], [0, 1, i*2]]), (600, 420))
            path = root / f"IMG_{i:04d}.jpg"
            self.assertTrue(cv2.imwrite(str(path), frame))
            records.append((path, 0., i/111320, 45., None))
        return records

    def wait_for(self, condition):
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            if condition():
                return
            time.sleep(.05)
        self.fail("Worker did not make progress within 15 seconds")

    def test_first_photo_starts_features_and_second_starts_matching(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            records = self.make_photos(root, 2)
            worker = InflightMatcher(root)
            db = root / ".inflight_matches.sqlite3"
            def count(table):
                if not db.exists():
                    return 0
                try:
                    with sqlite3.connect(db) as conn:
                        return conn.execute(f"SELECT COUNT(*) FROM {table}").fetchone()[0]
                except sqlite3.OperationalError:
                    return 0
            try:
                worker.submit(*records[0][:4])
                self.wait_for(lambda: count("features") == 1)
                self.assertEqual(count("pairs"), 0)
                worker.submit(*records[1][:4])
                self.wait_for(lambda: count("pairs") == 1)
            finally:
                worker.close()
            report = json.loads((root / "inflight_report.json").read_text())
            self.assertEqual(report["features_ready"], 2)
            self.assertEqual(report["accepted_pairs"], 1)
            with IncrementalCache(db) as cache, patch("orthomosaic.read_frame", side_effect=AssertionError("recomputed image")):
                a, _, _ = cache.feature(records[0][0])
                b, _, _ = cache.feature(records[1][0])
                with patch("orthomosaic.match_features", side_effect=AssertionError("recomputed pair")):
                    self.assertIsNotNone(cache.pair(a, b))

    def test_offline_matching_reuses_cached_positive_and_negative_pairs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            records = self.make_photos(root)
            area = GpsPolygon([(-.01, -.01), (-.01, .01), (.01, .01), (.01, -.01)])
            db = root / "cache.sqlite3"
            first = match_session(records, area, "none", .8, incremental_cache=db)
            with patch("orthomosaic.match_features", side_effect=AssertionError("pair was recomputed")):
                second = match_session(records, area, "none", .8, incremental_cache=db)
            self.assertEqual(first[0], second[0])
            self.assertEqual(len(first[1]), len(second[1]))
            for a, b in zip(first[1], second[1]):
                np.testing.assert_array_equal(a[2], b[2])
            with IncrementalCache(db) as cache:
                blank = root / "blank.jpg"
                cv2.imwrite(str(blank), np.zeros((420, 600, 3), np.uint8))
                a, _, _ = cache.feature(blank)
                b, _, _ = cache.feature(records[0][0])
                self.assertIsNone(cache.pair(a, b))
                with patch("orthomosaic.match_features", side_effect=AssertionError("negative pair was recomputed")):
                    self.assertIsNone(cache.pair(a, b))

    def test_changed_photo_invalidates_features_and_pairs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            records = self.make_photos(root, 1)
            path = records[0][0]
            with IncrementalCache(root / "cache.sqlite3") as cache:
                before, _, _ = cache.feature(path)
                cv2.imwrite(str(path), np.zeros((420, 600, 3), np.uint8))
                after, _, (_, descriptors) = cache.feature(path)
                self.assertNotEqual(before, after)
                self.assertEqual(len(descriptors), 0)


if __name__ == "__main__":
    unittest.main()
