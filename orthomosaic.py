"""Planar photogrammetric mosaic: lens correction, projective bundle, seams.

Unlike full ODM this estimates a single ground plane, not a dense 3D surface.
GPS fixes map orientation/scale after visual reconstruction, not every frame.
"""

import heapq
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np


def read_frame(path, profile="none", crop=0.8):
    if profile not in ("none", "synthetic-cgo3") or not 0 < crop <= 1:
        raise ValueError("Invalid camera correction profile or crop fraction")
    frame = cv2.imread(str(path))
    if frame is None:
        raise ValueError(f"Cannot read {path}")
    if profile == "synthetic-cgo3":
        # Invert the explicit remapping in camera_realism.py. This profile
        # applies ONLY to those synthetic images, not to an actual SIYI lens.
        from camera_realism import BARREL_K1, BARREL_K2, VIGNETTE_STRENGTH
        h, w = frame.shape[:2]
        full_w = w / crop
        margin = (full_w - w) / 2
        f = 0.85 * max(full_w, h)
        k = np.array([[f, 0, full_w / 2], [0, f, h / 2], [0, 0, 1]], np.float64)
        yy, xx = np.mgrid[:h, :w].astype(np.float32)
        desired = np.stack((xx + margin, yy), axis=-1)
        coords = cv2.undistortPoints(desired.reshape(-1, 1, 2), k,
                                    np.array([BARREL_K1, BARREL_K2, 0, 0, 0]), P=k).reshape(h, w, 2)
        # Undo known vignetting before geometry resampling.
        radius = np.sqrt(((xx + margin - full_w/2)/(full_w/2))**2 + ((yy-h/2)/(h/2))**2)
        gain = np.maximum(1 - VIGNETTE_STRENGTH * np.clip(radius, 0, 1.4)**2, 0.1)
        frame = np.clip(frame.astype(np.float32) / gain[..., None], 0, 255).astype(np.uint8)
        frame = cv2.remap(frame, coords[:, :, 0] - margin, coords[:, :, 1],
                          cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    return frame


def project(h, points):
    p = np.column_stack((points, np.ones(len(points)))) @ h.T
    return p[:, :2] / p[:, 2:3]


def normalization(shape):
    h, w = shape[:2]
    return np.array([[2/w, 0, -1], [0, 2/w, -h/w], [0, 0, 1]], np.float64)


def match_features(first, second, fast=False):
    """Shared RANSAC/quality checks for live and offline pair matching."""
    cv2.setRNGSeed(0)
    matcher = (cv2.FlannBasedMatcher(dict(algorithm=1, trees=4), dict(checks=96))
               if fast else cv2.BFMatcher())
    p, d = first
    q, e = second
    if d is None or e is None or min(len(d), len(e)) < 2:
        return None
    good = [a for a, b in matcher.knnMatch(d, e, k=2) if a.distance < 0.72*b.distance]
    if len(good) < 20:
        return None
    src = np.array([p[m.queryIdx] for m in good])
    dst = np.array([q[m.trainIdx] for m in good])
    hom, mask = cv2.findHomography(src, dst, cv2.RANSAC, 0.006, maxIters=4000, confidence=0.999)
    if hom is None:
        return None
    inliers = mask.ravel().astype(bool)
    if inliers.sum() < 18 or inliers.mean() < 0.35:
        return None
    src, dst = src[inliers], dst[inliers]
    if min(cv2.contourArea(cv2.convexHull(x.astype(np.float32))) for x in (src, dst)) < 0.08:
        return None
    if np.median(np.linalg.norm(project(np.linalg.inv(hom), dst)-src, axis=1)) > 0.008:
        return None
    # Fixed number per pair balances textured and less-textured regions.
    take = np.linspace(0, len(src)-1, 40).astype(int)
    return hom, src[take], dst[take]


def match_session(records, area, profile, crop, cache=None, fast=False, incremental_cache=None):
    if incremental_cache is None:
        return _match_session(records, area, profile, crop, cache, fast, None)
    from inflight_matching import IncrementalCache
    with IncrementalCache(incremental_cache, profile, crop) as store:
        result = _match_session(records, area, profile, crop, cache, fast, store)
        reuse = {"feature_cache_hits": store.feature_hits, "pair_cache_hits": store.pair_hits}
        Path(incremental_cache).with_name("inflight_reuse.json").write_text(json.dumps(reuse, indent=2)+"\n")
        print(f"Reused incremental work: {store.feature_hits} features, {store.pair_hits} pairs", flush=True)
        return result


def _match_session(records, area, profile, crop, cache, fast, store):
    # Cache only numeric arrays; metadata invalidates it when input changes.
    signature = json.dumps([(str(r[0]), r[0].stat().st_size, r[0].stat().st_mtime_ns)
                            for r in records] + [(profile, crop, "projective-fast-v2" if fast else "projective-v2")])
    if cache and cache.exists():
        with np.load(cache, allow_pickle=False) as data:
            if str(data["signature"]) == signature:
                print("Loading verified feature-match cache", flush=True)
                return [tuple(x) for x in data["shapes"]], [
                    (int(i), int(j), h, p, q) for i, j, h, p, q in
                    zip(data["i"], data["j"], data["homographies"], data["src"], data["dst"])]
    cv2.setRNGSeed(0)
    detector = cv2.SIFT_create(nfeatures=3500, contrastThreshold=0.025)
    features, shapes, feature_keys = [], [], []
    for i, record in enumerate(records):
        if store is not None:
            key, shape, feature = store.feature(record[0])
            feature_keys.append(key)
            shapes.append(shape)
            features.append(feature)
        else:
            frame = read_frame(record[0], profile, crop)
            shapes.append(frame.shape[:2])
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            keys, desc = detector.detectAndCompute(gray, None)
            points = np.array([key.pt for key in keys], np.float64).reshape(-1, 2)
            features.append((project(normalization(frame.shape), points), desc))
        if i % 50 == 0:
            print(f"Features: {i}/{len(records)}", flush=True)
    centers = np.array([area.to_local_m(r[1], r[2]) for r in records])
    pairs = set()
    for i in range(len(records)):
        distances = np.linalg.norm(centers - centers[i], axis=1)
        for j in np.argsort(distances)[1:(13 if fast else 26)]:
            if distances[j] < 120:
                pairs.add(tuple(sorted((i, int(j)))))
        for step in ((1, 2, 3, 5) if fast else (1, 2, 3, 5, 8, 15)):
            if i + step < len(records):
                pairs.add((i, i+step))
    # Visual retrieval closes loops between survey strips even when GPS-based
    # candidates miss overlapping views due to tilt or position errors.
    rng = np.random.default_rng(0)
    samples = [d[rng.choice(len(d), min(len(d), 50), replace=False)]
               for _, d in features if d is not None and len(d)]
    if not samples:
        raise ValueError("No visual features found in the photographs")
    sample = np.concatenate(samples)
    words = min(128, len(sample))
    _, _, vocabulary = cv2.kmeans(sample, words, None,
                                  (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1),
                                  1, cv2.KMEANS_PP_CENTERS)
    quantizer = cv2.BFMatcher()
    histograms = []
    for _, descriptors in features:
        assignments = [] if descriptors is None or len(descriptors) == 0 else [m.trainIdx for m in quantizer.match(descriptors, vocabulary)]
        histograms.append(np.bincount(assignments, minlength=words))
    histograms = np.array(histograms, np.float64)
    histograms *= np.log((len(records)+1)/(1+(histograms > 0).sum(axis=0))) + 1
    histograms = np.sqrt(histograms / np.maximum(histograms.sum(axis=1, keepdims=True), 1))
    similarity = histograms @ histograms.T
    for i in range(len(records)):
        for j in np.argsort(similarity[i])[-(9 if fast else 17):]:
            if i != j:
                pairs.add(tuple(sorted((i, int(j)))))
    edges = []
    for count, (i, j) in enumerate(sorted(pairs)):
        if count % 300 == 0:
            print(f"Projective matches: {count}/{len(pairs)}, accepted {len(edges)}", flush=True)
        pair = (store.pair(feature_keys[i], feature_keys[j], fast, (features[i], features[j]))
                if store is not None else match_features(features[i], features[j], fast))
        if pair is not None:
            edges.append((i, j, *pair))
    if not edges:
        raise ValueError("No reliable overlapping image pairs")
    if cache:
        np.savez_compressed(cache, signature=signature, shapes=shapes,
                            i=[e[0] for e in edges], j=[e[1] for e in edges],
                            homographies=[e[2] for e in edges], src=[e[3] for e in edges], dst=[e[4] for e in edges])
    return shapes, edges


def initial_poses(count, edges):
    neighbors = [[] for _ in range(count)]
    for index, (i, j, h, p, q) in enumerate(edges):
        score = min(cv2.contourArea(cv2.convexHull(x.astype(np.float32))) for x in (p, q))
        neighbors[i].append((j, index, score))
        neighbors[j].append((i, index, score))
    components, remaining = [], set(range(count))
    while remaining:
        component, stack = set(), [remaining.pop()]
        while stack:
            i = stack.pop()
            component.add(i)
            for j, _, _ in neighbors[i]:
                if j in remaining:
                    remaining.remove(j)
                    stack.append(j)
        components.append(component)
    connected = max(components, key=len)
    if len(connected) < 6:
        raise ValueError("Insufficient connected photographs for a coherent map")
    root = max(connected, key=lambda i: sum(s for _, _, s in neighbors[i]))
    poses = {root: np.eye(3)}
    queue = [(-s, root, j, index) for j, index, s in neighbors[root]]
    heapq.heapify(queue)
    while queue:
        _, parent, child, index = heapq.heappop(queue)
        if child in poses:
            continue
        i, j, h, _, _ = edges[index]
        poses[child] = poses[parent] @ (np.linalg.inv(h) if parent == i else h)
        poses[child] /= poses[child][2, 2]
        for nxt, idx, score in neighbors[child]:
            if nxt not in poses:
                heapq.heappush(queue, (-score, child, nxt, idx))
    print(f"Connected reconstruction: {len(poses)}/{count} images; reference {root}", flush=True)
    return poses, root


def projection_jacobian(h, points):
    u, v = points.T
    denominator = h[2, 0]*u + h[2, 1]*v + 1
    xy = project(h, points)
    jac = np.zeros((len(points), 2, 8))
    jac[:, 0, :3] = np.column_stack((u, v, np.ones(len(u)))) / denominator[:, None]
    jac[:, 1, 3:6] = jac[:, 0, :3]
    jac[:, :, 6] = -xy * (u / denominator)[:, None]
    jac[:, :, 7] = -xy * (v / denominator)[:, None]
    return xy, jac


def bundle(poses, root, edges, iterations=12):
    variables = {i: k for k, i in enumerate(sorted(set(poses)-{root}))}
    edges = [e for e in edges if e[0] in poses and e[1] in poses]
    size = len(variables)*8
    for iteration in range(iterations):
        normal = np.eye(size)*1e-5
        rhs = np.zeros(size)
        errors = []
        for i, j, _, p, q in edges:
            xp, ji = projection_jacobian(poses[i], p)
            xq, jj = projection_jacobian(poses[j], q)
            residual = xp-xq
            error = np.linalg.norm(residual, axis=1)
            errors.extend(error)
            weight = np.sqrt(np.minimum(1.0, 0.006 / np.maximum(error, 1e-8)))
            r = (residual*weight[:, None]).ravel()
            blocks = []
            for node, jac, sign in ((i, ji, 1), (j, jj, -1)):
                if node in variables:
                    sl = slice(variables[node]*8, (variables[node]+1)*8)
                    mat = (jac*weight[:, None, None]*sign).reshape(-1, 8)
                    normal[sl, sl] += mat.T@mat
                    rhs[sl] -= mat.T@r
                    blocks.append((sl, mat))
            if len(blocks) == 2:
                (si, a), (sj, b) = blocks
                cross = a.T@b
                normal[si, sj] += cross
                normal[sj, si] += cross.T
        ok, delta = cv2.solve(normal, rhs, flags=cv2.DECOMP_CHOLESKY)
        if not ok:
            raise ValueError("Projective bundle adjustment failed")
        delta = delta.ravel()
        # Damped steps guard the projective denominators during initialization.
        factor = min(1.0, 0.15 / max(np.max(np.abs(delta)), 1e-8))
        for node, k in variables.items():
            poses[node].flat[:8] += factor*delta[k*8:(k+1)*8]
        print(f"Bundle {iteration+1}: median residual {np.median(errors):.5f} reference units", flush=True)
        if np.max(np.abs(delta)) < 1e-6:
            break
    return poses


def select_spatial_frames(records, area, limit=220):
    """Cover the entire flight with farthest-point GPS sampling, retaining time order."""
    if limit < 6:
        raise ValueError("At least six frames must be retained")
    if len(records) <= limit:
        return list(records)
    centers = np.array([area.to_local_m(r[1], r[2]) for r in records])
    selected = [0]
    distance = np.full(len(records), np.inf)
    for _ in range(limit-1):
        distance = np.minimum(distance, np.sum((centers-centers[selected[-1]])**2, axis=1))
        distance[selected] = -1
        selected.append(int(np.argmax(distance)))
    return [records[i] for i in sorted(selected)]


def build_orthomosaic(records, area, resolution, profile, crop, out, hfov=114.6, fast=False):
    """Reconstruct and texture one connected ground plane from a session."""
    started = time.perf_counter()
    if len(records) < 6:
        raise ValueError("At least six photographs are needed for final reconstruction")
    if not (math.isfinite(resolution) and resolution > 0 and 0 < crop <= 1 and 0 < hfov < 180):
        raise ValueError("Invalid map resolution or camera geometry")
    if any(not all(math.isfinite(v) for v in record[1:4]) or record[3] <= 0 for record in records):
        raise ValueError("Photographs need finite GPS coordinates and positive relative altitude")
    source_count = len(records)
    if fast:
        records = select_spatial_frames(records, area)
        print(f"Fast map: {len(records)}/{source_count} spatially distributed images", flush=True)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(4)
    cache = out.parent / (".orthomosaic_fast_matches.npz" if fast else ".orthomosaic_matches.npz")
    incremental_cache = Path(records[0][0]).resolve().parent.parent / ".inflight_matches.sqlite3"
    shapes, edges = match_session(records, area, profile, crop, cache, fast=fast,
                                  incremental_cache=incremental_cache)
    matched = time.perf_counter()
    poses, root = initial_poses(len(records), edges)
    poses = bundle(poses, root, edges, iterations=24)
    residuals = np.concatenate([np.linalg.norm(project(poses[i], p)-project(poses[j], q), axis=1)
                                for i, j, _, p, q in edges if i in poses and j in poses]) * shapes[root][1]/2
    connected_frames = len(poses)
    world, gps_error, quality = metric_bundle(poses, edges, records, area, hfov, crop,
                                             out.parent, pixel_scale=shapes[root][1]/2)
    texture_poses = select_texture_poses(world, edges, shapes)
    aligned = time.perf_counter()
    if len(texture_poses) < 2:
        raise ValueError("Insufficient spatially supported views for a trustworthy map")
    report = render(texture_poses, records, shapes, area, resolution, profile, crop, out, hfov,
                     world_transform=np.eye(3), gps_error=gps_error)
    report.update(quality)
    report.update(source_frames=source_count, fast_mode=fast)
    finished = time.perf_counter()
    report["processing_seconds"] = round(finished-started, 3)
    report["stage_seconds"] = {"matching": round(matched-started, 3),
                               "alignment": round(aligned-matched, 3),
                               "texturing": round(finished-aligned, 3)}
    report["connected_frames"] = connected_frames
    report.update(input_frames=len(records), reference_frame=str(records[root][0]),
                  projective_initialization_median_error_px=float(np.median(residuals)),
                  texture_support_fraction_min=0.3,
                  excluded_frames=[str(r[0]) for i, r in enumerate(records) if i not in texture_poses])
    out.with_suffix(".json").write_text(json.dumps(report, indent=2)+"\n")
    print(f"Map processing time: {finished-started:.1f} s ({(finished-started)/60:.2f} min)", flush=True)
    return report


def metric_rectification(poses, hfov, crop):
    """Recover the ground normal from calibrated inter-camera homographies.

    The same plane normal must explain every view. Consensus resolves the
    two-fold homography decomposition ambiguity without assuming a level
    reference camera. Normalized cropped-image focal length is shared.
    """
    focal = 1 / (crop * math.tan(math.radians(hfov)/2))
    intrinsic = np.diag([focal, focal, 1.0])
    alternatives = []
    for hom in poses.values():
        _, _, translations, normals = cv2.decomposeHomographyMat(np.linalg.inv(hom), intrinsic)
        candidates = []
        for translation, normal in zip(translations, normals):
            if np.linalg.norm(translation) < 0.03:
                continue
            normal = normal.ravel()
            normal *= 1 if normal[2] > 0 else -1
            if normal[2] > 0.1:
                candidates.append(normal)
        if candidates:
            alternatives.append(np.array(candidates))
    if not alternatives:
        raise ValueError("Not enough camera translation to estimate the ground plane")
    candidates = np.concatenate(alternatives)
    scores = [np.median([np.min(1-options@normal) for options in alternatives]) for normal in candidates]
    best = candidates[np.argmin(scores)]
    selected = np.array([options[np.argmax(options@best)] for options in alternatives])
    normal = np.median(selected, axis=0)
    normal /= np.linalg.norm(normal)
    angular_error = float(np.median(np.degrees(np.arccos(np.clip(selected@normal, -1, 1)))))
    if angular_error > 15:
        raise ValueError("Inconsistent ground plane: use a full 3D reconstruction")
    east = np.cross([0, 1, 0], normal)
    east /= np.linalg.norm(east)
    south = np.cross(normal, east)
    rectification = np.array([east, south, normal]) @ np.linalg.inv(intrinsic)
    return rectification, intrinsic, angular_error


def georeference(poses, records, area, hfov=114.6, crop=0.8):
    rectification, intrinsic, angular_error = metric_rectification(poses, hfov, crop)
    ids = sorted(poses)
    centers = []
    for i in ids:
        # GPS measures the camera position, not the ground point hit by the
        # optical axis of a tilted camera. Decompose plane-to-image poses.
        calibrated = np.linalg.inv(intrinsic) @ np.linalg.inv(rectification @ poses[i])
        calibrated /= np.linalg.norm(calibrated[:, 0])
        rotation = np.column_stack((calibrated[:, 0], calibrated[:, 1],
                                     np.cross(calibrated[:, 0], calibrated[:, 1])))
        u, _, vt = np.linalg.svd(rotation)
        rotation = u @ vt
        centers.append((-rotation.T @ calibrated[:, 2])[:2])
    visual = np.array(centers)
    ground = np.array([area.to_local_m(records[i][1], records[i][2]) for i in ids])
    ground[:, 1] *= -1
    transform, inliers = cv2.estimateAffinePartial2D(visual, ground, method=cv2.RANSAC,
                                            ransacReprojThreshold=5, maxIters=10000,
                                            confidence=0.999, refineIters=20)
    if transform is None or inliers.sum() < 6:
        raise ValueError("Visual reconstruction cannot be anchored consistently to GPS")
    fitted = visual @ transform[:, :2].T + transform[:, 2]
    error = np.linalg.norm(fitted-ground, axis=1)
    print(f"Ground normal consistency: {angular_error:.2f} degrees", flush=True)
    print(f"GPS anchoring: {int(inliers.sum())}/{len(ids)} inliers, median {np.median(error):.2f} m", flush=True)
    return np.vstack((transform, [0, 0, 1])) @ rectification, float(np.median(error))


def metric_bundle(poses, edges, records, area, hfov, crop, work_dir, pixel_scale=512):
    """Fit calibrated cameras and shared ground points, with GPS constraints."""
    geo, _ = georeference(poses, records, area, hfov, crop)
    focal = 1/(crop*math.tan(math.radians(hfov)/2))
    intrinsic = np.diag([focal, focal, 1.])
    ids = sorted(poses)
    camera_index = {i: k for k, i in enumerate(ids)}
    cameras, gps = [], []
    for i in ids:
        b = np.linalg.inv(intrinsic) @ np.linalg.inv(geo @ poses[i])
        b /= np.linalg.norm(b[:, 0])
        if b[2, 2] < 0:
            b *= -1
        u, _, vt = np.linalg.svd(np.column_stack((b[:, 0], b[:, 1], np.cross(b[:, 0], b[:, 1]))))
        rotation = u @ vt
        if np.linalg.det(rotation) < 0:
            u[:, -1] *= -1
            rotation = u @ vt
        center = -rotation.T @ b[:, 2]
        cameras.append(np.r_[cv2.Rodrigues(rotation)[0].ravel(), center])
        east, north = area.to_local_m(records[i][1], records[i][2])
        gps.append((east, -north, -records[i][3]))
    # Exact SIFT coordinates identify the same feature in different pairs.
    # Union observations into tracks, rejecting tracks with multiple features
    # from the same photograph (a contradictory match).
    parent, observations, lookup = [], [], {}
    def observation(i, point):
        key = (i, *np.round(point, 7))
        if key not in lookup:
            lookup[key] = len(parent)
            parent.append(len(parent))
            observations.append((i, point))
        return lookup[key]
    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for i, j, _, p, q in edges:
        if i not in poses or j not in poses:
            continue
        for a, b in zip(p, q):
            ai, bi = observation(i, a), observation(j, b)
            parent[find(ai)] = find(bi)
    groups = {}
    for i, obs in enumerate(observations):
        groups.setdefault(find(i), []).append(obs)
    points, camera_ids, point_ids, measured = [], [], [], []
    for group in groups.values():
        if len(group) < 2 or len({i for i, _ in group}) != len(group):
            continue
        ground = np.array([project(geo @ poses[i], p[None])[0] for i, p in group])
        if not np.isfinite(ground).all():
            continue
        index = len(points)
        points.append(np.median(ground, axis=0))
        for i, point in group:
            camera_ids.append(camera_index[i])
            point_ids.append(index)
            measured.append(point)
    if len(points) < 20:
        raise ValueError("Insufficient consistent feature tracks for metric bundle adjustment")
    print(f"Metric bundle: {len(cameras)} cameras, {len(points)} tracks, {len(measured)} observations", flush=True)
    with tempfile.TemporaryDirectory(prefix="metric_bundle_", dir=work_dir) as tmp:
        source, target = Path(tmp)/"input.npz", Path(tmp)/"output.npz"
        np.savez_compressed(source, cameras=cameras, points=points, camera_ids=camera_ids,
                            point_ids=point_ids, observed=measured, gps=gps, focal=focal,
                            pixel_scale=pixel_scale)
        worker = Path(__file__).with_name("planar_bundle_worker.py")
        # The system stack is internally consistent even when user-site NumPy
        # shadows an older system SciPy. Only numeric NPZ crosses this boundary.
        probe = subprocess.run([sys.executable, "-c", "from scipy.optimize import least_squares"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        interpreter = [sys.executable] + (["-s"] if probe.returncode else [])
        worker_env = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
        subprocess.run([*interpreter, str(worker), str(source), str(target)], check=True, env=worker_env)
        with np.load(target, allow_pickle=False) as data:
            fitted = data["cameras"].copy()
            residuals = data["residuals"].copy()
            observation_counts = data["observation_counts"].copy()
            camera_error = data["per_camera_error"].copy()
            converged = bool(data["success"])
    world_poses = {}
    errors = np.linalg.norm(fitted[:, 3:]-np.array(gps), axis=1)
    for k, i in enumerate(ids):
        rotation = cv2.Rodrigues(fitted[k, :3])[0]
        tilt = math.degrees(math.acos(np.clip(abs(rotation[2, 2]), 0, 1)))
        if observation_counts[k] < 20 or camera_error[k] > 2.5 or errors[k] > 10 or tilt > 20:
            continue
        matrix = intrinsic @ np.column_stack((rotation[:, :2], -rotation @ fitted[k, 3:]))
        world_poses[i] = np.linalg.inv(matrix)
        world_poses[i] /= world_poses[i][2, 2]
    print(f"Metric result: median reprojection {np.median(np.linalg.norm(residuals, axis=1)):.2f} px, "
          f"camera GPS error {np.median(errors):.2f} m", flush=True)
    print(f"Cameras passing geometric quality checks: {len(world_poses)}/{len(ids)}", flush=True)
    if len(world_poses) < 6:
        raise ValueError("Too few cameras passed the metric reconstruction checks")
    quality = {"median_reprojection_error_px": float(np.median(np.linalg.norm(residuals, axis=1))),
               "metric_solver_converged": converged, "metric_input_cameras": len(ids),
               "metric_accepted_cameras": len(world_poses),
               "texture_rejected_frames": [str(records[i][0]) for i in ids if i not in world_poses]}
    return world_poses, float(np.median(errors)), quality


def supported_regions(poses, edges, tolerance_m=0.35):
    """Texture only regions supported by agreeing multi-view ground points."""
    points = {i: [] for i in poses}
    for i, j, _, p, q in edges:
        if i not in poses or j not in poses:
            continue
        good = np.linalg.norm(project(poses[i], p)-project(poses[j], q), axis=1) < tolerance_m
        points[i].extend(p[good])
        points[j].extend(q[good])
    hulls = {}
    for i, p in points.items():
        if len(p) >= 12:
            hull = cv2.convexHull(np.array(p, np.float32)).reshape(-1, 2)
            if cv2.contourArea(hull) > 0.1:
                hulls[i] = hull
    return hulls


def select_texture_poses(poses, edges, shapes, min_fraction=0.3):
    hulls = supported_regions(poses, edges)
    selected = {i: pose for i, pose in poses.items() if i in hulls
                and cv2.contourArea(hulls[i]) >= min_fraction*4*shapes[i][0]/shapes[i][1]}
    print(f"Views with broad geometric support: {len(selected)}/{len(poses)}", flush=True)
    return selected


def render(poses, records, shapes, area, resolution, profile, crop, out, hfov=114.6,
           world_transform=None, gps_error=None, support_polygons=None):
    if world_transform is None:
        geo, gps_error = georeference(poses, records, area, hfov, crop)
    else:
        geo = world_transform
    min_e, min_n, max_e, max_n = area.bounding_box_m()
    margin = 5
    width = int(math.ceil((max_e-min_e+2*margin)*resolution))
    height = int(math.ceil((max_n-min_n+2*margin)*resolution))
    if width*height > 50_000_000:
        raise ValueError("Map exceeds 50 megapixels; lower --resolution")
    canvas_transform = np.array([[resolution, 0, (-min_e+margin)*resolution],
                                 [0, resolution, (max_n+margin)*resolution], [0, 0, 1]])
    candidates = []
    best = np.zeros((height, width), np.float32)
    labels = np.full((height, width), -1, np.int32)
    # Build footprints once, retaining only finite, nonfolded, plausible views.
    for i in sorted(poses):
        if support_polygons is not None and i not in support_polygons:
            continue
        h, w = shapes[i]
        matrix = canvas_transform @ geo @ poses[i] @ normalization((h, w))
        corners = np.array([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]], np.float64)
        den = np.column_stack((corners, np.ones(4))) @ matrix[2]
        if np.min(den)*np.max(den) <= 0 or np.min(np.abs(den))/np.max(np.abs(den)) < 0.25:
            continue
        projected = project(matrix, corners)
        if not np.isfinite(projected).all():
            continue
        footprint = abs(cv2.contourArea(projected.astype(np.float32))) / resolution**2
        # A strongly oblique / unstable footprint should not supply texture.
        nominal = (2*records[i][3]*math.tan(math.radians(hfov)/2))**2 * crop**2 * h/w
        if not 0.2*nominal < footprint < 2.5*nominal:
            continue
        x0, y0 = np.maximum(np.floor(projected.min(axis=0)).astype(int), 0)
        x1, y1 = np.minimum(np.ceil(projected.max(axis=0)).astype(int)+1, (width, height))
        if x1-x0 < 10 or y1-y0 < 10:
            continue
        shift = np.array([[1, 0, -x0], [0, 1, -y0], [0, 0, 1]])
        matrix = shift @ matrix
        yy, xx = np.mgrid[:h, :w].astype(np.float32)
        radius = ((xx-w/2)/(w/2))**2 + ((yy-h/2)/(w/2))**2
        score = np.exp(-2*radius).astype(np.float32)
        score[:12] = score[-12:] = 0
        score[:, :12] = score[:, -12:] = 0
        if support_polygons is not None:
            support = np.zeros((h, w), np.uint8)
            polygon = project(np.linalg.inv(normalization((h, w))), support_polygons[i])
            cv2.fillConvexPoly(support, np.round(polygon).astype(np.int32), 255)
            support = cv2.dilate(support, np.ones((21, 21), np.uint8))
            score *= support / 255
        warped_score = cv2.warpPerspective(score, matrix, (x1-x0, y1-y0))
        corner = (int(x0), int(y0))
        roi = best[y0:y1, x0:x1]
        use = warped_score > roi
        labels[y0:y1, x0:x1][use] = len(candidates)
        roi[use] = warped_score[use]
        # Only retain geometry. Hundreds of float score maps formerly consumed GBs.
        candidates.append((i, matrix, corner, warped_score.shape))
    if not candidates:
        raise ValueError("No geometrically valid footprints")
    counts = np.bincount(labels[labels >= 0], minlength=len(candidates))
    threshold = max(4, min(400, width*height//1000))
    selected = [c for k, c in enumerate(candidates) if counts[k] >= threshold]
    if not selected:
        raise ValueError("Insufficient image coverage at the requested resolution")
    print(f"Texturing: {len(selected)} source views, {width}x{height} pixels", flush=True)
    del best, labels, warped_score, roi, use, score, yy, xx, radius
    def load_tile(k):
        i, matrix, corner, (tile_h, tile_w) = selected[k]
        frame = read_frame(records[i][0], profile, crop)
        h, w = shapes[i]
        yy, xx = np.mgrid[:h, :w].astype(np.float32)
        score = np.exp(-2*(((xx-w/2)/(w/2))**2 + ((yy-h/2)/(w/2))**2)).astype(np.float32)
        score[:12] = score[-12:] = 0
        score[:, :12] = score[:, -12:] = 0
        if support_polygons is not None:
            support = np.zeros((h, w), np.uint8)
            polygon = project(np.linalg.inv(normalization((h, w))), support_polygons[i])
            cv2.fillConvexPoly(support, np.round(polygon).astype(np.int32), 255)
            support = cv2.dilate(support, np.ones((21, 21), np.uint8))
            score *= support/255
        mask = np.uint8(cv2.warpPerspective(score, matrix, (tile_w, tile_h)) > 0.025)*255
        image = cv2.warpPerspective(frame, matrix, (tile_w, tile_h), flags=cv2.INTER_LINEAR,
                                   borderMode=cv2.BORDER_REFLECT)
        return image, mask
    result, coverage = blend_streamed(load_tile, [c[2] for c in selected], width, height,
                                      protect_parallax=True)
    encoded = np.dstack((result, coverage)) if Path(out).suffix.lower() == ".png" else result
    if not cv2.imwrite(str(out), encoded):
        raise OSError(f"Cannot save {out}")
    report = {"connected_frames": len(poses), "texture_frames": len(selected),
              "gps_median_error_m": gps_error, "coverage_fraction": float(np.mean(coverage > 0)),
              "width": width, "height": height, "camera_profile": profile,
              "pixel_size_m": 1/resolution,
              "local_origin_lat_lon": [area.ref_lat, area.ref_lon],
              "upper_left_east_north_m": [min_e-margin, max_n+margin],
              "parallax_protection": "single central source per supported disagreement region",
              "model": "single ground plane; not a dense 3D ODM reconstruction"}
    Path(out).with_suffix(".json").write_text(json.dumps(report, indent=2)+"\n")
    print(f"Saved {out}; coverage {report['coverage_fraction']:.1%}", flush=True)
    return report


def blend_tiles(images, masks, corners, width, height):
    """Exposure compensation, graph-cut seams, then multiband compositing."""
    return blend_streamed(lambda k: (images[k].copy(), masks[k]), corners, width, height)


def exposure_block_size(shapes, budget=2048):
    size = 32
    while sum(math.ceil(h/size)*math.ceil(w/size) for h, w in shapes) > max(budget, len(shapes)):
        size *= 2
    return size


def parallax_owners(images, masks, corners, width, height):
    """Keep displaced objects coherent using one view per disagreement region.

    This detects disagreement, not semantic trees or their height. No pixels
    are invented; oversized regions without a covering source stay untouched.
    """
    total = np.zeros((height, width), np.float32)
    squared = np.zeros_like(total)
    count = np.zeros_like(total)
    for im, mask, (x, y) in zip(images, masks, corners):
        h, w = min(im.shape[0], height-y), min(im.shape[1], width-x)
        gray = cv2.cvtColor(im[:h, :w], cv2.COLOR_BGR2GRAY).astype(np.float32)
        valid = mask[:h, :w] > 0
        total[y:y+h, x:x+w] += gray*valid
        squared[y:y+h, x:x+w] += gray*gray*valid
        count[y:y+h, x:x+w] += valid
    variance = squared/np.maximum(count, 1)-(total/np.maximum(count, 1))**2
    disagreement = np.uint8((variance > 22**2) & (count >= 3))*255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    disagreement = cv2.morphologyEx(disagreement, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(disagreement, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    disagreement[:] = 0
    cv2.drawContours(disagreement, contours, -1, 255, cv2.FILLED)
    disagreement = cv2.dilate(disagreement, np.ones((3, 3), np.uint8))
    n, labels, stats, centers = cv2.connectedComponentsWithStats(disagreement)
    owners = np.full((height, width), -1, np.int32)
    protected = 0
    for label in range(1, n):
        left, top, w, h, area = stats[label]
        if area < 12:
            continue
        best, chosen = -1., None
        for k, (mask, (x, y)) in enumerate(zip(masks, corners)):
            x0, y0 = max(left, x), max(top, y)
            x1, y1 = min(left+w, x+mask.shape[1], width), min(top+h, y+mask.shape[0], height)
            if x1 <= x0 or y1 <= y0:
                continue
            valid = (labels[y0:y1, x0:x1] == label) & (mask[y0-y:y1-y, x0-x:x1-x] > 0)
            fraction = np.count_nonzero(valid)/area
            if fraction < .98:
                continue
            cx, cy = centers[label]
            radius = ((cx-x-mask.shape[1]/2)/(mask.shape[1]/2))**2 + ((cy-y-mask.shape[0]/2)/(mask.shape[0]/2))**2
            score = fraction*np.exp(-2*radius)
            if score > best:
                best, chosen = score, (k, x0, y0, x1, y1, valid)
        if chosen is not None:
            k, x0, y0, x1, y1, valid = chosen
            owners[y0:y1, x0:x1][valid] = k
            protected += 1
    print(f"Parallax protection: {protected} coherent source regions", flush=True)
    return owners


def blend_streamed(load_tile, corners, width, height, protect_parallax=False):
    """Keep only small seam previews; load full-size tiles one at a time."""
    # Estimate spatial brightness compensation from overlaps, then choose seams
    # through low-disagreement regions and blend frequency bands near them.
    seam_scale = min(1.0, 700/max(width, height))
    small_images, small_masks = [], []
    for k in range(len(corners)):
        im, mask = load_tile(k)
        small = cv2.resize(im, (max(1, round(im.shape[1]*seam_scale)),
                               max(1, round(im.shape[0]*seam_scale))))
        small_images.append(small)
        small_masks.append(cv2.resize(mask, (small.shape[1], small.shape[0]), interpolation=cv2.INTER_NEAREST))
        del im, mask
    print(f"Exposure and seams: {len(corners)} small previews", flush=True)
    small_corners = [(round(x*seam_scale), round(y*seam_scale)) for x, y in corners]
    # Block exposure compensation solves a dense system over ALL image blocks.
    # A fixed 32px block across hundreds of previews can require >10 GB.
    # Bound the system to 2048 variables, coarsening brightness correction only.
    block_size = exposure_block_size([im.shape[:2] for im in small_images])
    compensator = cv2.detail_BlocksGainCompensator(block_size, block_size)
    print(f"Exposure block size: {block_size}px (bounded equation count)", flush=True)
    compensator.feed(small_corners, small_images, small_masks)
    for k, (im, mask, corner) in enumerate(zip(small_images, small_masks, small_corners)):
        compensator.apply(k, corner, im, mask)
    owners = None
    if protect_parallax:
        small_owners = parallax_owners(small_images, small_masks, small_corners,
                                      max(x+im.shape[1] for im, (x, y) in zip(small_images, small_corners)),
                                      max(y+im.shape[0] for im, (x, y) in zip(small_images, small_corners)))
        # Use the same scale as the tile projections, not a stretched canvas.
        owners = cv2.resize(small_owners, None, fx=1/seam_scale, fy=1/seam_scale,
                            interpolation=cv2.INTER_NEAREST)
        padded = np.full((height, width), -1, np.int32)
        hh, ww = min(height, owners.shape[0]), min(width, owners.shape[1])
        padded[:hh, :ww] = owners[:hh, :ww]
        owners = padded
        sharp = np.zeros((height, width, 3), np.uint8)
        sharp_mask = np.zeros((height, width), np.uint8)
    print("Exposure complete; finding seams", flush=True)
    seam_finder = cv2.detail_GraphCutSeamFinder("COST_COLOR_GRAD")
    umat_masks = [cv2.UMat(m) for m in small_masks]
    seam_finder.find([im.astype(np.float32) for im in small_images], small_corners, umat_masks)
    blender = cv2.detail_MultiBandBlender()
    blender.setNumBands(5)
    blender.prepare((0, 0, width, height))
    del small_images, small_masks
    for k, corner in enumerate(corners):
        im, mask = load_tile(k)
        if k % 25 == 0:
            print(f"Blending tile {k+1}/{len(corners)}", flush=True)
        compensator.apply(k, corner, im, mask)
        if owners is not None:
            x, y = corner
            h, w = im.shape[:2]
            use = (owners[y:y+h, x:x+w] == k) & (mask > 0)
            sharp[y:y+h, x:x+w][use] = im[use]
            sharp_mask[y:y+h, x:x+w][use] = 255
        seam = cv2.dilate(umat_masks[k].get(), np.ones((3, 3), np.uint8))
        seam = cv2.resize(seam, (im.shape[1], im.shape[0]), interpolation=cv2.INTER_LINEAR)
        seam = cv2.bitwise_and(seam, mask)
        blender.feed(im.astype(np.int16), seam, corner)
        del im, mask, seam
    result, coverage = blender.blend(None, None)
    result = np.clip(result, 0, 255).astype(np.uint8)
    if owners is not None:
        alpha = np.minimum(cv2.distanceTransform(sharp_mask, cv2.DIST_L2, 3)/2, 1)[..., None]
        result = np.uint8(np.rint(result*(1-alpha)+sharp*alpha))
    result[coverage == 0] = 0
    return result, coverage
