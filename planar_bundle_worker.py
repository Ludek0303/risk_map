"""Sparse metric bundle adjustment; executable with the system Python (-s).

Input/output are numeric NPZ arrays, so this worker is independent of Gazebo,
EXIF and the caller's NumPy ABI. It needs NumPy and SciPy, no network service.
"""

import argparse

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation


def optimize(data):
    cameras = data["cameras"]
    points = data["points"]
    ci, pi = data["camera_ids"], data["point_ids"]
    observed, gps = data["observed"], data["gps"]
    focal = float(data["focal"])
    pixel_scale = float(data["pixel_scale"])
    initial_rotations = Rotation.from_rotvec(cameras[:, :3]).as_matrix()
    initial_xyz = np.column_stack((points[pi], np.zeros(len(pi)))) - cameras[ci, 3:]
    initial_local = np.einsum("nij,nj->ni", initial_rotations[ci], initial_xyz)
    initial_prediction = focal*initial_local[:, :2]/np.maximum(initial_local[:, 2:3], .1)
    valid = (initial_local[:, 2] > 1) & (np.linalg.norm(initial_prediction-observed, axis=1)*pixel_scale < 100)
    track_counts = np.bincount(pi[valid], minlength=len(points))
    valid &= track_counts[pi] >= 2
    print(f"Metric observations with valid initial geometry: {valid.sum()}/{len(valid)}", flush=True)
    ci, pi, observed = ci[valid], pi[valid], observed[valid]
    ncam = len(cameras)
    nobs = len(observed)
    sparsity = lil_matrix((2*nobs+3*ncam, cameras.size+points.size), dtype=np.int8)
    for axis in range(2):
        rows = 2*np.arange(nobs)+axis
        for k in range(6):
            sparsity[rows, 6*ci+k] = 1
        for k in range(2):
            sparsity[rows, cameras.size+2*pi+k] = 1
    for i in range(ncam):
        for axis in range(3):
            sparsity[2*nobs+3*i+axis, 6*i+3+axis] = 1
    count = 0
    def residual(vector):
        nonlocal count
        cam = vector[:cameras.size].reshape(-1, 6)
        xy = vector[cameras.size:].reshape(-1, 2)
        rotations = Rotation.from_rotvec(cam[:, :3]).as_matrix()
        xyz = np.column_stack((xy[pi], np.zeros(nobs))) - cam[ci, 3:]
        local = np.einsum("nij,nj->ni", rotations[ci], xyz)
        depth = np.maximum(local[:, 2:3], 0.1)
        prediction = focal*local[:, :2]/depth
        # Both residual groups have interpretable scales: pixels and metres.
        reprojection = (prediction-observed)*pixel_scale
        position = (cam[:, 3:]-gps) / 1.0
        count += 1
        if count % 250 == 0:
            print(f"Metric bundle: median reprojection {np.median(np.linalg.norm(reprojection, axis=1)):.2f} px; "
                  f"GPS {np.median(np.linalg.norm(position, axis=1)):.2f} m", flush=True)
        return np.concatenate((reprojection.ravel(), position.ravel()))
    initial = np.concatenate((cameras.ravel(), points.ravel()))
    result = least_squares(residual, initial, jac_sparsity=sparsity.tocsr(),
                           method="trf", loss="soft_l1", f_scale=2,
                           x_scale="jac", ftol=1e-5, xtol=1e-7,
                           max_nfev=300, tr_options={"atol": 1e-5, "btol": 1e-5, "maxiter": 100}, verbose=1)
    cam = result.x[:cameras.size].reshape(-1, 6)
    final_residuals = residual(result.x)[:2*nobs].reshape(-1, 2)
    per_camera_error = np.array([np.median(np.linalg.norm(final_residuals[ci == i], axis=1))
                                 if np.any(ci == i) else np.inf for i in range(ncam)])
    return {"cameras": cam, "points": result.x[cameras.size:].reshape(-1, 2),
            "residuals": final_residuals, "observation_counts": np.bincount(ci, minlength=ncam),
            "per_camera_error": per_camera_error,
            "success": result.success, "message": result.message}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input")
    parser.add_argument("output")
    args = parser.parse_args()
    with np.load(args.input, allow_pickle=False) as data:
        result = optimize(data)
    np.savez_compressed(args.output, **result)


if __name__ == "__main__":
    main()
