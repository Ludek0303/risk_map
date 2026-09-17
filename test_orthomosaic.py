import tempfile
import unittest
import weakref
import math
from pathlib import Path

import cv2
import numpy as np

from area import GpsPolygon
from orthomosaic import blend_tiles, blend_streamed, bundle, exposure_block_size, georeference, metric_bundle, parallax_owners, project, read_frame, select_texture_poses, select_spatial_frames


class OrthomosaicTests(unittest.TestCase):
    def test_displaced_crown_uses_one_complete_source(self):
        images = []
        for x in (28, 36, 44):
            im = np.full((80, 80, 3), 130, np.uint8)
            cv2.circle(im, (x, 40), 12, (10, 10, 10), -1)
            images.append(im)
        masks = [np.full((80, 80), 255, np.uint8) for _ in images]
        owners = parallax_owners(images, masks, [(0, 0)]*3, 80, 80)
        self.assertEqual(len(np.unique(owners[owners >= 0])), 1)
        self.assertTrue(np.all(owners[28:53, 28:45] >= 0))
        self.assertTrue(np.all(owners[:15] == -1))
        result, coverage = blend_streamed(lambda k: (images[k].copy(), masks[k]),
                                         [(0, 0)]*3, 80, 80, protect_parallax=True)
        self.assertTrue(np.all(coverage > 0))
        # Inside the protected region there should be a sharp source, not an average of shifted crowns.
        self.assertLess(np.count_nonzero((result[35:46, 25:49, 0] > 25) &
                                        (result[35:46, 25:49, 0] < 110)), 8)

    def test_exposure_system_is_bounded_for_hundreds_of_views(self):
        shapes = [(220, 320)] * 376
        size = exposure_block_size(shapes)
        blocks = sum(math.ceil(h/size)*math.ceil(w/size) for h, w in shapes)
        self.assertLessEqual(blocks, 2048)
        self.assertGreater(size, 32)
        self.assertEqual(exposure_block_size([(64, 64)]*3), 32)

    def test_streaming_releases_full_tiles_before_loading_next(self):
        previous = []
        calls = []
        def load(k):
            self.assertTrue(all(ref() is None for ref in previous))
            im = np.full((64, 64, 3), 100+k*10, np.uint8)
            mask = np.full((64, 64), 255, np.uint8)
            previous[:] = [weakref.ref(im), weakref.ref(mask)]
            calls.append(k)
            return im, mask
        result, coverage = blend_streamed(load, [(0, 0), (24, 0), (48, 0)], 112, 64)
        self.assertEqual(calls, [0, 1, 2, 0, 1, 2])
        self.assertEqual(result.shape, (64, 112, 3))
        self.assertTrue(np.all(coverage > 0))

    def test_fast_selection_covers_all_strips_and_keeps_time_order(self):
        area = GpsPolygon([(-.01, -.01), (-.01, .01), (.01, .01), (.01, -.01)])
        records = [(Path(f"{i*50+j}.jpg"), j/111320, i*20/111320, 45, None)
                   for i in range(8) for j in range(50)]
        selected = select_spatial_frames(records, area, 80)
        self.assertEqual(len(selected), 80)
        self.assertEqual(len(set(r[2] for r in selected)), 8)
        indices = [int(r[0].stem) for r in selected]
        self.assertEqual(indices, sorted(set(indices)))
        self.assertEqual(records, select_spatial_frames(records, area, 500))

    def cameras(self):
        intrinsics = np.diag([1., 1., 1.])
        homographies, records = [], []
        for i, (east, south) in enumerate(((0, 0), (8, 0), (16, 0), (0, 8), (8, 8), (16, 8))):
            rotation, _ = cv2.Rodrigues(np.array([0.12 + i*.01, -.06, i*.025]))
            center = np.array([east, south, -20.])
            hom = intrinsics @ np.column_stack((rotation[:, :2], -rotation@center))
            homographies.append(hom)
            records.append((Path(f"{i}.jpg"), -south/111320, east/111320, 20., None))
        poses = {i: homographies[0] @ np.linalg.inv(h) for i, h in enumerate(homographies)}
        poses = {i: h/h[2, 2] for i, h in poses.items()}
        return homographies, poses, records

    def test_tilted_cameras_rectify_to_metric_ground(self):
        homographies, poses, records = self.cameras()
        area = GpsPolygon([(-.001, -.001), (-.001, .001), (.001, .001), (.001, -.001)])
        transform, error = georeference(poses, records, area, 90, 1)
        points = np.array([[1., 2.], [12., 5.], [4., 10.]])
        projected = project(transform, project(homographies[0], points))
        np.testing.assert_allclose(projected, points, atol=.01)
        self.assertLess(error, .01)

    def test_bundle_corrects_perspective_and_closes_loops(self):
        hs, truth, _ = self.cameras()
        rng = np.random.default_rng(42)
        ground = rng.uniform([0, 0], [16, 8], (50, 2))
        edges = []
        for i in range(6):
            for j in range(i+1, 6):
                p, q = project(hs[i], ground), project(hs[j], ground)
                edges.append((i, j, hs[j] @ np.linalg.inv(hs[i]), p, q))
        noisy = {i: h.copy() for i, h in truth.items()}
        for i in range(1, 6):
            noisy[i].flat[:8] += rng.normal(0, .003, 8)
        fitted = bundle(noisy, 0, edges, 10)
        for i in range(6):
            np.testing.assert_allclose(project(fitted[i], project(hs[i], ground)),
                                       project(hs[0], ground), atol=1e-5)

    def test_metric_bundle_recovers_known_camera_and_ground_geometry(self):
        hs, poses, records = self.cameras()
        rng = np.random.default_rng(21)
        ground = rng.uniform([0, 0], [16, 8], (50, 2))
        edges = [(i, j, hs[j]@np.linalg.inv(hs[i]), project(hs[i], ground), project(hs[j], ground))
                 for i in range(6) for j in range(i+1, 6)]
        area = GpsPolygon([(-.001, -.001), (-.001, .001), (.001, .001), (.001, -.001)])
        with tempfile.TemporaryDirectory() as tmp:
            world, gps_error, quality = metric_bundle(poses, edges, records, area, 90, 1, tmp)
        self.assertEqual(len(world), 6)
        self.assertLess(gps_error, .01)
        self.assertLess(quality["median_reprojection_error_px"], .01)
        for i in world:
            np.testing.assert_allclose(project(world[i], project(hs[i], ground)), ground, atol=.01)

    def test_synthetic_vignette_profile_is_explicit_and_effective(self):
        from camera_realism import _apply_vignette
        image = np.full((240, 400, 3), 120, np.uint8)
        vignetted = _apply_vignette(image)[:, 40:-40]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "frame.png"
            cv2.imwrite(str(path), vignetted)
            np.testing.assert_array_equal(read_frame(path), vignetted)
            corrected = read_frame(path, "synthetic-cgo3", .8)
        self.assertLess(corrected.std(), 1)
        self.assertLess(abs(corrected.mean()-120), 2)

    def test_blending_removes_exposure_seam_without_losing_coverage(self):
        left = np.full((120, 160, 3), 100, np.uint8)
        right = np.full((120, 160, 3), 140, np.uint8)
        masks = [np.full((120, 160), 255, np.uint8) for _ in range(2)]
        result, coverage = blend_tiles([left, right], masks, [(0, 0), (80, 0)], 240, 120)
        self.assertTrue((coverage > 0).all())
        # Before correction the seam has a 40-level jump. Check the actual
        # radiometric continuity through the overlap, not an implementation flag.
        scanline = result[60, 40:200, 0].astype(float)
        self.assertLess(np.max(np.abs(np.diff(scanline))), 4)

    def test_narrow_matched_patch_cannot_supply_an_entire_frame(self):
        wide = np.array([(x, y) for x in np.linspace(-.8, .8, 5) for y in np.linspace(-.8, .8, 5)])
        narrow = wide * [.2, 1.]
        poses = {i: np.eye(3) for i in range(3)}
        edges = [(0, 1, np.eye(3), wide, wide), (1, 2, np.eye(3), narrow, narrow)]
        selected = select_texture_poses(poses, edges, [(100, 100)]*3)
        self.assertEqual(set(selected), {0, 1})


if __name__ == "__main__":
    unittest.main()
