"""Planar photogrammetric mosaic: lens correction, projective bundle, seams.

Unlike full ODM this estimates a single ground plane, not a dense 3D surface.
GPS fixes map orientation/scale after visual reconstruction, not every frame.
"""

import heapq
import json
import math
from pathlib import Path

import cv2
import numpy as np


def read_frame(path, profile="none", crop=0.8):
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


def match_session(records, area, profile, crop, cache=None):
    # Cache only numeric arrays; metadata invalidates it when input changes.
    signature = json.dumps([(str(r[0]), r[0].stat().st_size, r[0].stat().st_mtime_ns)
                            for r in records] + [(profile, crop, "projective-v1")])
    if cache and cache.exists():
        with np.load(cache, allow_pickle=False) as data:
            if str(data["signature"]) == signature:
                print("Loading verified feature-match cache", flush=True)
                return [tuple(x) for x in data["shapes"]], [
                    (int(i), int(j), h, p, q) for i, j, h, p, q in
                    zip(data["i"], data["j"], data["homographies"], data["src"], data["dst"])]
    cv2.setRNGSeed(0)
    detector = cv2.SIFT_create(nfeatures=3500, contrastThreshold=0.025)
    features, shapes = [], []
    for i, record in enumerate(records):
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
        for j in np.argsort(distances)[1:26]:
            if distances[j] < 120:
                pairs.add(tuple(sorted((i, int(j)))))
        for step in (1, 2, 3, 5, 8, 15):
            if i + step < len(records):
                pairs.add((i, i+step))
    edges = []
    matcher = cv2.BFMatcher()
    for count, (i, j) in enumerate(sorted(pairs)):
        if count % 300 == 0:
            print(f"Projective matches: {count}/{len(pairs)}, accepted {len(edges)}", flush=True)
        p, d = features[i]
        q, e = features[j]
        if d is None or e is None or min(len(d), len(e)) < 2:
            continue
        good = [a for a, b in matcher.knnMatch(d, e, k=2) if a.distance < 0.72*b.distance]
        if len(good) < 20:
            continue
        src = np.array([p[m.queryIdx] for m in good])
        dst = np.array([q[m.trainIdx] for m in good])
        hom, mask = cv2.findHomography(src, dst, cv2.RANSAC, 0.006, maxIters=4000, confidence=0.999)
        if hom is None:
            continue
        inliers = mask.ravel().astype(bool)
        if inliers.sum() < 18 or inliers.mean() < 0.35:
            continue
        src, dst = src[inliers], dst[inliers]
        if min(cv2.contourArea(cv2.convexHull(x.astype(np.float32))) for x in (src, dst)) < 0.08:
            continue
        if np.median(np.linalg.norm(project(np.linalg.inv(hom), dst)-src, axis=1)) > 0.008:
            continue
        # Fixed number per pair balances textured and less-textured regions.
        take = np.linspace(0, len(src)-1, 40).astype(int)
        edges.append((i, j, hom, src[take], dst[take]))
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
    if len(connected) < 4:
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


def georeference(poses, records, area):
    ids = sorted(poses)
    visual = np.array([project(poses[i], np.zeros((1, 2)))[0] for i in ids])
    ground = np.array([area.to_local_m(records[i][1], records[i][2]) for i in ids])
    ground[:, 1] *= -1
    transform, inliers = cv2.estimateAffine2D(visual, ground, method=cv2.RANSAC,
                                            ransacReprojThreshold=8, maxIters=10000,
                                            confidence=0.999, refineIters=20)
    if transform is None or inliers.sum() < 6:
        raise ValueError("Visual reconstruction cannot be anchored consistently to GPS")
    fitted = visual @ transform[:, :2].T + transform[:, 2]
    error = np.linalg.norm(fitted-ground, axis=1)
    print(f"GPS anchoring: {int(inliers.sum())}/{len(ids)} inliers, median {np.median(error):.2f} m", flush=True)
    return np.vstack((transform, [0, 0, 1])), float(np.median(error))


def render(poses, records, shapes, area, resolution, profile, crop, out):
    geo, gps_error = georeference(poses, records, area)
    min_e, min_n, max_e, max_n = area.bounding_box_m()
    margin = 5
    width = int(math.ceil((max_e-min_e+2*margin)*resolution))
    height = int(math.ceil((max_n-min_n+2*margin)*resolution))
    if width*height > 50_000_000:
        raise ValueError("Map exceeds 50 megapixels; lower --resolution")
    canvas_transform = np.array([[resolution, 0, (-min_e+margin)*resolution],
                                 [0, resolution, (max_n+margin)*resolution], [0, 0, 1]])
    candidates = []
    # Build footprints once, retaining only finite, nonfolded, plausible views.
    for i in sorted(poses):
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
        nominal = (2*records[i][3]*math.tan(math.radians(114.6)/2))**2 * crop**2 * h/w
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
        warped_score = cv2.warpPerspective(score, matrix, (x1-x0, y1-y0))
        candidates.append((i, matrix, (int(x0), int(y0)), warped_score))
    if not candidates:
        raise ValueError("No geometrically valid footprints")
    # Select useful source frames before costly exposure and seam optimization.
    best = np.zeros((height, width), np.float32)
    labels = np.full((height, width), -1, np.int32)
    for k, (_, _, (x, y), score) in enumerate(candidates):
        h, w = score.shape
        roi = best[y:y+h, x:x+w]
        use = score > roi
        labels[y:y+h, x:x+w][use] = k
        roi[use] = score[use]
    counts = np.bincount(labels[labels >= 0], minlength=len(candidates))
    selected = [c for k, c in enumerate(candidates) if counts[k] >= 400]
    print(f"Texturing: {len(selected)} source views, {width}x{height} pixels", flush=True)
    images, masks, corners = [], [], []
    for i, matrix, corner, score in selected:
        frame = read_frame(records[i][0], profile, crop)
        h, w = score.shape
        image = cv2.warpPerspective(frame, matrix, (w, h), flags=cv2.INTER_LINEAR,
                                    borderMode=cv2.BORDER_REFLECT)
        mask = np.uint8(score > 0.025)*255
        images.append(image)
        masks.append(mask)
        corners.append(corner)
    # Estimate spatial brightness compensation from overlaps, then choose seams
    # through low-disagreement regions and blend frequency bands near them.
    seam_scale = min(1.0, 700/max(width, height))
    small_images = [cv2.resize(im, None, fx=seam_scale, fy=seam_scale) for im in images]
    small_masks = [cv2.resize(m, (im.shape[1], im.shape[0]), interpolation=cv2.INTER_NEAREST)
                   for m, im in zip(masks, small_images)]
    small_corners = [(round(x*seam_scale), round(y*seam_scale)) for x, y in corners]
    compensator = cv2.detail_ExposureCompensator.createDefault(cv2.detail.ExposureCompensator_GAIN_BLOCKS)
    compensator.feed(small_corners, small_images, small_masks)
    for k, (im, mask, corner) in enumerate(zip(small_images, small_masks, small_corners)):
        compensator.apply(k, corner, im, mask)
    seam_finder = cv2.detail_GraphCutSeamFinder("COST_COLOR_GRAD")
    umat_masks = [cv2.UMat(m) for m in small_masks]
    seam_finder.find([im.astype(np.float32) for im in small_images], small_corners, umat_masks)
    blender = cv2.detail_MultiBandBlender()
    blender.setNumBands(5)
    blender.prepare((0, 0, width, height))
    for k, (im, mask, corner) in enumerate(zip(images, masks, corners)):
        compensator.apply(k, corner, im, mask)
        seam = cv2.dilate(umat_masks[k].get(), np.ones((3, 3), np.uint8))
        seam = cv2.resize(seam, (im.shape[1], im.shape[0]), interpolation=cv2.INTER_LINEAR)
        seam = cv2.bitwise_and(seam, mask)
        blender.feed(im.astype(np.int16), seam, corner)
    result, coverage = blender.blend(None, None)
    result = np.clip(result, 0, 255).astype(np.uint8)
    result[coverage == 0] = 0
    if not cv2.imwrite(str(out), result):
        raise OSError(f"Cannot save {out}")
    report = {"connected_frames": len(poses), "texture_frames": len(selected),
              "gps_median_error_m": gps_error, "coverage_fraction": float(np.mean(coverage > 0)),
              "width": width, "height": height, "camera_profile": profile,
              "model": "single ground plane; not a dense 3D ODM reconstruction"}
    Path(out).with_suffix(".json").write_text(json.dumps(report, indent=2)+"\n")
    print(f"Saved {out}; coverage {report['coverage_fraction']:.1%}", flush=True)
    return report
