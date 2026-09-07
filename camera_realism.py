"""Camera-realism post-processing for Gazebo-rendered frames (OpenCV only).

Gazebo's simulated camera is an ideal, noise-free pinhole render. The real
camera used for test flights on this airframe is a SIYI A2 mini: a 1/2.7"
2MP "Starlight" CMOS behind a 160deg ultra-wide lens, streamed out as
encoded 1080p video. That combination has real-world artifacts a synthetic
render simply doesn't produce - barrel distortion, vignetting, chromatic
aberration, sensor noise (more visible on a small, low-light-tuned sensor
than on a large one), lens softness, AE/AWB hunting, and video-codec
compression blur. All of these affect how well OpenDroneMap can actually
match features between frames, so running captures through this before
saving keeps testing honest to what the real camera will hand the mapping
pipeline, instead of tuning against unrealistically clean input.

The parameters below are reasonable approximations for a small ultra-wide
action-style CMOS camera, not measurements taken off this specific unit -
adjust them if side-by-side footage against the real A2 mini says
otherwise.
"""

from __future__ import annotations

import cv2
import numpy as np

# --- Lens: residual barrel distortion + lateral chromatic aberration -------
# SIYI advertises in-camera "distortion correction" for its 160deg lens, but
# ultra-wide optics like this still show mild residual barrel curvature and
# color fringing toward the edges even after correction.
BARREL_K1 = -0.14
BARREL_K2 = 0.03
CHROMATIC_ABERRATION_PX = 1.6  # radial R/B channel offset at the frame edge

# --- Vignetting -------------------------------------------------------------
VIGNETTE_STRENGTH = 0.30  # fractional darkening at the corners

# --- Lens softness: real optics never resolve as sharply as a ray-traced
# pinhole render, especially toward the edges of a wide lens -----------------
CENTER_SOFTNESS_SIGMA = 0.6
EDGE_SOFTNESS_SIGMA = 1.6

# --- Sensor noise: read noise (constant) + shot noise (signal-dependent).
# More visible here than on a large-sensor camera - a 1/2.7" 2MP "Starlight"
# sensor runs at meaningfully higher analog gain than a full-frame sensor to
# get its low-light sensitivity, and that gain shows up as noise.
READ_NOISE_SIGMA = 2.5   # 0-255 scale
SHOT_NOISE_SCALE = 0.035

# --- Auto-exposure / auto-white-balance hunting: a real camera doesn't hold
# a perfectly flat exposure/white-balance from one frame to the next --------
AE_JITTER_STD = 0.04    # fractional brightness jitter, per frame
AWB_JITTER_STD = 0.02   # fractional per-channel gain jitter, per frame

# --- Video-codec compression: the real camera streams encoded 1080p video,
# not lossless frames. A JPEG re-encode at a middling quality is a cheap
# OpenCV-only stand-in for the blockiness/blur that leaves behind. ----------
COMPRESSION_JPEG_QUALITY = 70

_rng = np.random.default_rng()


def _apply_lens_distortion(frame: np.ndarray) -> np.ndarray:
    """Barrel-distorts the frame and offsets the R/B channels radially
    (lateral chromatic aberration) relative to G."""
    h, w = frame.shape[:2]
    f = 0.85 * max(w, h)  # approximate focal length in pixels
    camera_matrix = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], dtype=np.float64)
    dist = np.array([BARREL_K1, BARREL_K2, 0, 0, 0], dtype=np.float64)

    # Standard trick to *add* synthetic distortion to a clean image: build
    # the undistortion maps for the target distCoeffs and remap through them.
    map1, map2 = cv2.initUndistortRectifyMap(
        camera_matrix, dist, None, camera_matrix, (w, h), cv2.CV_32FC1
    )

    cx, cy = w / 2.0, h / 2.0
    ca_shift = CHROMATIC_ABERRATION_PX / (0.5 * max(w, h))
    out = np.empty_like(frame)
    for channel, radial_sign in ((0, -1.0), (1, 0.0), (2, 1.0)):  # BGR: B in, R out
        plane = cv2.remap(frame[:, :, channel], map1, map2, cv2.INTER_LINEAR)
        if radial_sign != 0.0:
            scale = 1.0 + radial_sign * ca_shift
            m = np.array([[scale, 0, cx * (1 - scale)], [0, scale, cy * (1 - scale)]], dtype=np.float64)
            plane = cv2.warpAffine(plane, m, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        out[:, :, channel] = plane
    return out


def _radial_map(w: int, h: int) -> np.ndarray:
    """Per-pixel distance from center, normalized so the frame edges sit at ~1.0."""
    y, x = np.mgrid[0:h, 0:w].astype(np.float32)
    cx, cy = w / 2.0, h / 2.0
    return np.sqrt(((x - cx) / cx) ** 2 + ((y - cy) / cy) ** 2)


def _apply_vignette(frame: np.ndarray) -> np.ndarray:
    h, w = frame.shape[:2]
    r = _radial_map(w, h)
    mask = np.clip(1.0 - VIGNETTE_STRENGTH * np.clip(r, 0, 1.4) ** 2, 0, 1)[:, :, None]
    return (frame.astype(np.float32) * mask).astype(np.uint8)


def _apply_softness(frame: np.ndarray) -> np.ndarray:
    h, w = frame.shape[:2]
    center = cv2.GaussianBlur(frame, (0, 0), CENTER_SOFTNESS_SIGMA)
    edge = cv2.GaussianBlur(frame, (0, 0), EDGE_SOFTNESS_SIGMA)
    blend = np.clip(_radial_map(w, h), 0, 1)[:, :, None]
    return (center.astype(np.float32) * (1 - blend) + edge.astype(np.float32) * blend).astype(np.uint8)


def _apply_sensor_noise(frame: np.ndarray) -> np.ndarray:
    f = frame.astype(np.float32)
    shot = _rng.normal(0.0, 1.0, f.shape).astype(np.float32) * np.sqrt(np.clip(f, 0, None)) * SHOT_NOISE_SCALE
    read = _rng.normal(0.0, READ_NOISE_SIGMA, f.shape).astype(np.float32)
    return np.clip(f + shot + read, 0, 255).astype(np.uint8)


def _apply_ae_awb_jitter(frame: np.ndarray) -> np.ndarray:
    brightness = 1.0 + _rng.normal(0.0, AE_JITTER_STD)
    gains = 1.0 + _rng.normal(0.0, AWB_JITTER_STD, size=3)
    f = frame.astype(np.float32) * brightness * gains[None, None, :]
    return np.clip(f, 0, 255).astype(np.uint8)


def _apply_compression(frame: np.ndarray) -> np.ndarray:
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, COMPRESSION_JPEG_QUALITY])
    if not ok:
        return frame
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR)


def apply_camera_realism(frame: np.ndarray) -> np.ndarray:
    """Run a Gazebo-rendered BGR frame through the SIYI-A2-mini-like
    degradation pipeline: lens distortion + chromatic aberration -> vignette
    -> lens softness -> sensor noise -> AE/AWB jitter -> codec compression.
    """
    frame = _apply_lens_distortion(frame)
    frame = _apply_vignette(frame)
    frame = _apply_softness(frame)
    frame = _apply_sensor_noise(frame)
    frame = _apply_ae_awb_jitter(frame)
    frame = _apply_compression(frame)
    return frame


def main() -> None:
    import argparse

    from gimbal_control import GimbalController
    from gz_camera import DEFAULT_MODEL, DEFAULT_WORLD, GzCameraStream, build_camera_topic

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default=None, help="Full Gazebo image topic. Overrides --world/--model.")
    parser.add_argument("--world", default=DEFAULT_WORLD)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--point-down", action="store_true", help="Point the gimbal straight down via MAVLink first.")
    args = parser.parse_args()
    topic = args.topic or build_camera_topic(args.world, args.model)

    if args.point_down:
        print("Pointing gimbal down ...")
        with GimbalController() as gimbal:
            gimbal.point_down()

    print(f"Subscribing to: {topic}")
    stream = GzCameraStream(topic=topic)

    window = "raw (left) vs SIYI-A2-mini-like (right) - q to quit"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    try:
        while True:
            frame = stream.read(timeout=1.0)
            if frame is None:
                continue
            processed = apply_camera_realism(frame)
            cv2.imshow(window, np.hstack([frame, processed]))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
