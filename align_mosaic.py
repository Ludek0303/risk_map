"""Globally fit 2D similarity poses from SIFT matches and GPS priors.

This assumes approximately flat terrain. It is not a 3D reconstruction.
"""

import math

import cv2
import numpy as np


def align_frames(records, area, hfov, crop):
    cv2.setRNGSeed(0)
    detector = cv2.SIFT_create(nfeatures=2500)
    matcher = cv2.BFMatcher()
    features, shapes = [], []
    centers, priors = [], []
    for path, lat, lon, alt, heading in records:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        h, w = gray.shape
        resize = min(1.0, 800 / w)
        small = cv2.resize(gray, None, fx=resize, fy=resize)
        keys, desc = detector.detectAndCompute(small, None)
        points = np.array([k.pt for k in keys], np.float64).reshape(-1, 2)
        points /= resize
        points = (points - (w / 2, h / 2)) / (w / 2)
        features.append((points, desc))
        shapes.append((h, w))
        east, north = area.to_local_m(lat, lon)
        centers.append((east, -north))
        radius = alt * math.tan(math.radians(hfov) / 2) * crop
        angle = math.radians(heading)
        priors.append((radius * math.cos(angle), radius * math.sin(angle), east, -north))
    centers, priors = np.array(centers), np.array(priors)
    pairs = set()
    for i in range(len(records)):
        distance = np.linalg.norm(centers - centers[i], axis=1)
        for j in np.argsort(distance)[1:11]:
            if distance[j] < 90:
                pairs.add(tuple(sorted((i, int(j)))))
        for offset in (1, 3, 8):
            if i + offset < len(records):
                pairs.add((i, i + offset))
    edges = []
    for index, (i, j) in enumerate(sorted(pairs)):
        p, d = features[i]
        q, e = features[j]
        if d is None or e is None or min(len(d), len(e)) < 2:
            continue
        good = [a for a, b in matcher.knnMatch(d, e, k=2) if a.distance < 0.75 * b.distance]
        if len(good) < 14:
            continue
        src = np.array([p[m.queryIdx] for m in good])
        dst = np.array([q[m.trainIdx] for m in good])
        transform, mask = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC,
                                                     ransacReprojThreshold=0.008,
                                                     maxIters=3000, confidence=0.995)
        if transform is None:
            continue
        valid = mask.ravel().astype(bool)
        if valid.sum() < 12 or valid.mean() < 0.35:
            continue
        src, dst = src[valid], dst[valid]
        if min(np.std(src, axis=0).min(), np.std(dst, axis=0).min()) < 0.08:
            continue
        # Uniformly limit each edge's influence; dense textured patches should
        # not outweigh the rest of the survey simply by having more features.
        take = np.linspace(0, len(src) - 1, min(30, len(src))).astype(int)
        edges.append((i, j, src[take], dst[take]))
        if index % 200 == 0:
            print(f"Matching: {index}/{len(pairs)} pairs, {len(edges)} accepted", flush=True)
    if not edges:
        raise ValueError("No reliable image matches; cannot align this session")

    # Solve all poses together: x=a*u-b*v+tx, y=b*u+a*v+ty.
    # Reweight residuals so wrong correspondences cannot drag the entire map.
    n = len(records) * 4
    solution = priors.ravel().copy()
    for iteration in range(4):
        normal = np.zeros((n, n), np.float64)
        rhs = np.zeros(n, np.float64)
        def add(ids, values, target, weight):
            ids = np.asarray(ids)
            values = np.asarray(values)
            normal[np.ix_(ids, ids)] += weight * np.outer(values, values)
            rhs[ids] += weight * values * target
        for i, prior in enumerate(priors):
            for k in range(4):
                add([4*i+k], [1], prior[k], 1 / (20 if k < 2 else 4) ** 2)
        errors = []
        for i, j, src, dst in edges:
            ids = list(range(4*i, 4*i+4)) + list(range(4*j, 4*j+4))
            for (u, v), (s, t) in zip(src, dst):
                vx = [u, -v, 1, 0, -s, t, -1, 0]
                vy = [v, u, 0, 1, -t, -s, 0, -1]
                error = math.hypot(np.dot(vx, solution[ids]), np.dot(vy, solution[ids]))
                weight = 1.0 if iteration == 0 else min(1.0, 1.0 / max(error, 1e-6))
                add(ids, vx, 0, weight)
                add(ids, vy, 0, weight)
                errors.append(error)
        ok, fitted = cv2.solve(normal, rhs, flags=cv2.DECOMP_CHOLESKY)
        if not ok:
            raise ValueError("Global alignment is singular")
        solution = fitted.ravel()
        print(f"Alignment {iteration+1}/4: median match residual before solve {np.median(errors):.2f} m", flush=True)
    poses = solution.reshape(-1, 4)
    # Reject untrustworthy or disconnected views instead of stamping them over
    # the aligned result. Large relief and tilted frames break the flat model.
    support = np.zeros(len(records), int)
    neighbors = [set() for _ in records]
    for i, j, src, dst in edges:
        def project(points, pose):
            a, b, x, y = pose
            return points @ np.array([[a, b], [-b, a]]) + (x, y)
        residual = np.linalg.norm(project(src, poses[i]) - project(dst, poses[j]), axis=1)
        if np.median(residual) < 2:
            support[i] += 1
            support[j] += 1
            neighbors[i].add(j)
            neighbors[j].add(i)
    usable = (support >= 2)
    scale_ratio = np.linalg.norm(poses[:, :2], axis=1) / np.linalg.norm(priors[:, :2], axis=1)
    usable &= (scale_ratio > 0.65) & (scale_ratio < 1.4)
    # Tiny independent islands have too little visual evidence. Larger
    # components retain their GPS anchors, but may show seams between groups.
    remaining = set(np.flatnonzero(usable))
    components = []
    while remaining:
        pending = [remaining.pop()]
        component = set(pending)
        while pending:
            for j in neighbors[pending.pop()] & remaining:
                remaining.remove(j)
                component.add(j)
                pending.append(j)
        components.append(component)
    if components:
        supported = set().union(*(c for c in components if len(c) >= 4))
        usable &= np.array([i in supported for i in range(len(records))])
        print(f"Consistent component sizes: {sorted(map(len, components), reverse=True)}", flush=True)
        if sum(len(c) >= 4 for c in components) > 1:
            print("WARNING: separate image groups are positioned by GPS; seams between groups remain possible.", flush=True)
    print(f"Aligned views: {usable.sum()}/{len(records)}; accepted pairs: {len(edges)}", flush=True)
    if usable.sum() < 2:
        raise ValueError("Too few consistently aligned frames")
    return poses, shapes, usable
