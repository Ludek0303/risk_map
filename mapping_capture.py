"""Autonomous geotagged photo capture and real-time mapping.

Runs independently of whatever flies the drone (a QGroundControl waypoint
mission, manual RC control, whatever) - it never sends flight commands
itself, it only watches MAVLink telemetry and the Gazebo camera feed:

  * receives frames either from the Gazebo camera or an RTSP camera stream
  * runs simulated frames through camera_realism.py's OpenCV pipeline
    (lens distortion, vignetting, sensor noise, compression, ...) so the
    imagery approximates what the real SIYI A2 mini test camera would
    actually hand the mapping pipeline, not a noise-free synthetic render
  * takes a GPS mapping area (a polygon, see area.py) given up front, and
    waits for the vehicle to actually be inside it - capture only starts
    (and pauses again if it leaves) once it's over the area to map,
    regardless of when/whether it's armed elsewhere in transit
  * while armed and inside the area, saves a geotagged JPEG every
    --capture-interval-m of ground travel (the standard photogrammetry
    front-overlap trigger - distance based, not time based, so a hover
    doesn't spam duplicates and a fast pass doesn't undersample)
  * skips the shot if the vehicle is tilted past --max-tilt-deg: the sim
    gimbal (like the real SIYI A2 mini) only stabilizes pitch, so a banked
    turn rolls the camera straight with the airframe and points it at the
    horizon instead of the ground - a frame like that isn't nadir and
    corrupts both the live mosaic and OpenDroneMap if captured
  * places every captured frame onto a live GPS-direct-georectified mosaic
    (see live_mosaic.py) and re-saves it immediately - no feature matching
    or bundle adjustment, so it's far less precise than a full
    photogrammetry reconstruction, but it updates in milliseconds per frame
    instead of minutes, so a real terrain map exists throughout the flight
    even on modest onboard hardware (e.g. a Raspberry Pi 5) within a short
    mission's time budget
  * on landing, aligns saved clean mapping frames using image features and
    GPS priors, writing a separate lossless final_map.png (disable with
    --no-final-map); the live preview remains available during processing
  * optionally (--odm), also reruns OpenDroneMap (Docker) in the background
    every --odm-update-interval images and once more on landing, for a
    slower but far more precise reconstruction alongside the live mosaic -
    realistically needs minutes per run and a machine with Docker, so it's
    off by default

Requires: piexif (`pip install --user --break-system-packages piexif`).
--odm additionally needs Docker with the opendronemap/odm image (pulled on
first use).
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import piexif
from pymavlink import mavutil

from area import GpsPolygon
from camera_realism import apply_camera_realism
from gimbal_control import DEFAULT_CONNECTION, GimbalController
from inflight_matching import InflightMatcher
from live_mosaic import LiveMosaic
from rtsp_camera import RtspCameraStream, validate_rtsp_url

try:
    from gz_camera import DEFAULT_MODEL, DEFAULT_WORLD, GzCameraStream, build_camera_topic
except ModuleNotFoundError:
    # A real-camera laptop does not need Gazebo Transport installed.
    DEFAULT_WORLD = "baylands"
    DEFAULT_MODEL = "x500_gimbal_0"
    GzCameraStream = None

    def build_camera_topic(world_name: str, model_name: str) -> str:
        return f"/world/{world_name}/model/{model_name}/link/camera_link/sensor/camera/image"

EARTH_RADIUS_M = 6371000.0
MIN_IMAGES_FOR_ODM = 5
MIN_IMAGES_FOR_FINAL_MAP = 6

# x500_gimbal cgo3 camera, from PX4-Autopilot's gimbal/model.sdf
# <horizontal_fov>2.0</horizontal_fov> (radians).
CAMERA_HFOV_DEG = 114.6
# _crop_airframe() below trims this fraction of the width off *each* side.
AIRFRAME_CROP_FRACTION = 0.10
# ODM/OpenSfM needs a focal length to scale the reconstruction correctly -
# without it, it falls back to a generic guess that can be wildly wrong
# (producing a reconstruction hundreds of times too large/small). Expressing
# it as the 35mm-equivalent (relative to the standard 36mm reference width)
# avoids also needing to fabricate a physical sensor size.
CAMERA_FOCAL_LENGTH_35MM = round(18.0 / math.tan(math.radians(CAMERA_HFOV_DEG) / 2))
CAMERA_MAKE = "Gazebo"
CAMERA_MODEL = "x500_gimbal_cgo3"


def mission_planner_udp_connection(host: str, port: int) -> str:
    """Return the receive-only MAVLink endpoint forwarded by Mission Planner."""
    if not isinstance(port, int) or isinstance(port, bool) or not 1 <= port <= 65535:
        raise ValueError("Mission Planner UDP port must be an integer from 1 to 65535")
    if host not in {"127.0.0.1", "localhost"}:
        raise ValueError("Mission Planner forwarding must use localhost, not a network-facing address")
    # Mission Planner remains the only process opening LR900's serial device.
    return f"udpin:127.0.0.1:{port}"


def lr900_usb_connection(port: str, baud: int) -> tuple[str, int]:
    """Validate the serial endpoint; pymavlink opens this as MAVLink over UART."""
    if not port or not port.strip():
        raise ValueError("LR900 USB port cannot be empty")
    if not isinstance(baud, int) or isinstance(baud, bool) or baud <= 0:
        raise ValueError("LR900 baud rate must be a positive integer")
    return port.strip(), baud


def _planar_distance_m(lat1, lon1, lat2, lon2) -> float:
    """Flat-earth approximation - fine at mapping-flight scales."""
    lat_rad = math.radians((lat1 + lat2) / 2.0)
    dx = math.radians(lon2 - lon1) * math.cos(lat_rad) * EARTH_RADIUS_M
    dy = math.radians(lat2 - lat1) * EARTH_RADIUS_M
    return math.hypot(dx, dy)


def _deg_to_dms_rational(deg: float):
    deg = abs(deg)
    d = int(deg)
    m_full = (deg - d) * 60
    m = int(m_full)
    s = (m_full - m) * 60
    return ((d, 1), (m, 1), (int(s * 100), 100))


def _crop_airframe(frame):
    """The gimbal-down camera's 114.6deg HFOV is wide enough to catch the
    vehicle's own landing gear/arms in the left/right edges of every frame
    (fixed image position, since the legs are rigidly attached to the
    airframe rather than the gimbal). Left uncropped, these static
    "features" match perfectly - and with zero parallax - between every
    single image pair, which corrupts OpenSfM's reconstruction (confirmed:
    removing them fixed a bootstrap failure / wildly wrong scale estimate).
    """
    h, w = frame.shape[:2]
    margin = int(w * AIRFRAME_CROP_FRACTION)
    return frame[:, margin:w - margin]


def _write_geotagged_jpeg(
    frame, path: Path, lat: float, lon: float, alt_m: float, heading_deg: float,
    camera_make: str = CAMERA_MAKE, camera_model: str = CAMERA_MODEL,
    focal_length_35mm: int = CAMERA_FOCAL_LENGTH_35MM,
) -> None:
    """frame must already be cropped (see _crop_airframe) - the caller does
    this once and reuses the same cropped frame for the live mosaic."""
    ok, encoded = cv2.imencode(".jpg", frame)
    if not ok:
        raise RuntimeError("cv2.imencode failed")

    exif_dict = {
        "0th": {
            piexif.ImageIFD.Make: camera_make,
            piexif.ImageIFD.Model: camera_model,
        },
        "Exif": {
            piexif.ExifIFD.FocalLengthIn35mmFilm: focal_length_35mm,
        },
        "GPS": {
            piexif.GPSIFD.GPSLatitudeRef: "N" if lat >= 0 else "S",
            piexif.GPSIFD.GPSLatitude: _deg_to_dms_rational(lat),
            piexif.GPSIFD.GPSLongitudeRef: "E" if lon >= 0 else "W",
            piexif.GPSIFD.GPSLongitude: _deg_to_dms_rational(lon),
            piexif.GPSIFD.GPSAltitudeRef: 0 if alt_m >= 0 else 1,
            piexif.GPSIFD.GPSAltitude: (int(abs(alt_m) * 100), 100),
            # True vehicle heading at capture time ("T" = relative to true,
            # not magnetic, north) - not needed by mapping_capture.py itself
            # (the live mosaic gets it straight from telemetry in-memory),
            # but without it a captured session can't be reprocessed/tested
            # later except by crudely guessing heading from GPS deltas.
            piexif.GPSIFD.GPSImgDirectionRef: "T",
            piexif.GPSIFD.GPSImgDirection: (int((heading_deg % 360) * 100), 100),
        },
    }
    exif_bytes = piexif.dump(exif_dict)

    # Write to a temp file and rename into place atomically: an in-flight
    # ODM run (see OdmRunner) reads whatever's in images/ *while* new
    # photos keep landing there, and a reader can catch a same-named file
    # mid-write otherwise.
    tmp_path = path.with_suffix(".tmp")
    tmp_path.write_bytes(encoded.tobytes())
    piexif.insert(exif_bytes, str(tmp_path))
    tmp_path.replace(path)


def run_odm(
    session_dir: Path, feature_quality: str, orthophoto_resolution: float = 5.0,
    label: str = "final",
) -> None:
    # docker -v needs an absolute host path - a relative one (e.g. plain
    # "captures") gets silently treated as a *named volume* instead of a
    # bind mount, so the container sees an empty dir instead of our images.
    session_dir = session_dir.resolve()
    captures_dir = session_dir.parent
    project_name = session_dir.name
    images_dir = session_dir / "images"

    n_images = len(list(images_dir.glob("*.jpg")))
    print(
        f"\n[ODM:{label}] {n_images} images captured so far. "
        f"Starting reconstruction for '{project_name}' ..."
    )

    cmd = [
        "docker", "run", "--rm",
        "-v", f"{captures_dir}:/datasets/code",
        "opendronemap/odm",
        "--project-path", "/datasets/code",
        project_name,
        "--fast-orthophoto",
        "--feature-quality", feature_quality,
        # ODM uses centimetres per pixel (5 cm/px = 20 px/m).
        # Larger values reduce output dimensions and memory consumption.
        "--orthophoto-resolution", str(orthophoto_resolution),
    ]
    print(f"[ODM:{label}] {' '.join(cmd)}")

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for line in proc.stdout:
        print(f"[ODM:{label}] {line.rstrip()}")
    proc.wait()

    ortho = session_dir / "odm_orthophoto" / "odm_orthophoto.tif"
    if proc.returncode == 0 and ortho.exists():
        print(f"\n[ODM:{label}] Done. Orthophoto: {ortho}")
    else:
        print(f"\n[ODM:{label}] Finished with exit code {proc.returncode} (check output above).")


class OdmRunner:
    """Runs ODM reconstructions in a background thread so image capture
    never has to block on one.

    Only one reconstruction runs at a time; trigger() is a no-op (returns
    False) while one is already in flight rather than queuing another, so
    an update always starts over the freshest image set instead of a stale
    one queued minutes earlier. Each run overwrites session_dir's previous
    output with a fresh reconstruction over the (by then larger) image set,
    so the map on disk keeps getting more complete/accurate as the flight
    continues.
    """

    def __init__(self, feature_quality: str, orthophoto_resolution: float) -> None:
        self._feature_quality = feature_quality
        self._orthophoto_resolution = orthophoto_resolution
        self._lock = threading.Lock()
        self._busy = False
        self._thread: Optional[threading.Thread] = None

    def trigger(self, session_dir: Path, label: str) -> bool:
        """Start a reconstruction in the background. Returns False without
        doing anything if one is already running - the caller should just
        try again once more images have come in."""
        with self._lock:
            if self._busy:
                return False
            self._busy = True

        def _run() -> None:
            try:
                run_odm(session_dir, self._feature_quality, self._orthophoto_resolution, label=label)
            finally:
                with self._lock:
                    self._busy = False

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()
        return True

    def wait_idle(self) -> None:
        """Block until any in-progress reconstruction finishes."""
        thread = self._thread
        if thread is not None:
            thread.join()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connection", default=DEFAULT_CONNECTION)
    parser.add_argument("--mission-planner-udp-port", type=int, default=None, metavar="PORT",
                        help="Receive local MAVLink telemetry forwarded by Mission Planner. "
                             "Mission Planner remains the only reader of LR900's COM port.")
    parser.add_argument("--mission-planner-host", default="127.0.0.1", metavar="HOST",
                        help="Mission Planner forwarding host; only 127.0.0.1/localhost is accepted.")
    parser.add_argument("--lr900-usb-port", default=None, metavar="PORT",
                        help="Serial port of the LR900 ground radio, e.g. /dev/ttyUSB0 or COM3. "
                             "Reads MAVLink directly without Mission Planner.")
    parser.add_argument("--lr900-baud", type=int, default=57600, metavar="BAUD",
                        help="LR900 USB baud rate (default: 57600).")
    parser.add_argument("--relay-gimbal-control", action="store_true",
                        help="Allow gimbal commands through Mission Planner's UDP relay. Off by default.")
    parser.add_argument("--world", default=DEFAULT_WORLD)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--rtsp-url", default=None, metavar="URL",
                        help="RTSP source for real camera frames, e.g. rtsp://192.168.144.25:8554/main.264")
    parser.add_argument("--rtsp-startup-timeout", type=float, default=15.0, metavar="SECONDS",
                        help="Fail if the RTSP stream supplies no frame within this time (default: 15).")
    parser.add_argument("--camera-hfov-deg", type=float, default=CAMERA_HFOV_DEG,
                        help="Camera horizontal field of view used for map geometry (default: 114.6).")
    parser.add_argument("--camera-make", default=None, metavar="MAKE",
                        help="EXIF camera manufacturer; defaults to SIYI for --rtsp-url.")
    parser.add_argument("--camera-model", default=None, metavar="MODEL",
                        help="EXIF camera model; defaults to RTSP-main.264 for --rtsp-url.")
    parser.add_argument("--capture-interval-m", type=float, default=5.0)
    parser.add_argument("--min-altitude-m", type=float, default=1.0,
                        help="Minimum relative altitude for photos; use 95%% of survey altitude to skip climb/descent.")
    parser.add_argument(
        "--max-tilt-deg", type=float, default=12.0,
        help="Skip capturing while the vehicle's roll or pitch exceeds this many "
             "degrees - the gimbal only stabilizes pitch (single-axis, matching "
             "the real SIYI A2 mini), so a banked turn tilts the camera off nadir "
             "and a shot taken then is bad for both the live mosaic and ODM.",
    )
    parser.add_argument("--output-dir", default="captures")
    parser.add_argument("--no-final-map", action="store_true",
                        help="Skip feature-aligned final_map.png generation after landing.")
    parser.add_argument("--final-map-fast", action="store_true",
                        help="Use up to 220 distributed photographs for faster final reconstruction.")
    parser.add_argument("--no-inflight-matching", action="store_true",
                        help="Disable background feature/pair computation during flight.")
    parser.add_argument(
        "--polygon", default=None, metavar="AREA.json",
        help="Path to a JSON file listing the GPS area to map, as a flat list of "
             "[lat, lon] vertices (at least 3), e.g. "
             '[[52.2297,21.0122],[52.2299,21.0130],[52.2291,21.0135]]. Required '
             "unless --run-odm-on is used. Capturing (and in-flight map updates) "
             "only run while the vehicle's GPS position is inside this polygon.",
    )
    parser.add_argument(
        "--mosaic-resolution", type=float, default=8.0,
        help="Pixels per meter for the live GPS-direct-georectified mosaic "
             "(live_mosaic.py) that's rebuilt after every captured image. "
             "This is the map you get during/immediately after the flight "
             "without Docker/ODM.",
    )
    parser.add_argument(
        "--mosaic-out", default=None, metavar="PATH",
        help="Where to (re)save the live mosaic. Defaults to <session_dir>/live_map.jpg.",
    )
    parser.add_argument(
        "--odm", action="store_true",
        help="Also run OpenDroneMap (Docker) in the background: every "
             "--odm-update-interval images during the flight, and once more "
             "on landing, for a slower but far more precise reconstruction "
             "alongside the live mosaic. Needs Docker and realistically "
             "minutes per run - off by default since it usually can't keep "
             "up with a short mission (use --run-odm-on to do this later, "
             "offline, on any already-captured session instead).",
    )
    parser.add_argument(
        "--odm-update-interval", type=int, default=20,
        help="With --odm: rerun ODM in the background every N new images "
             "while still flying. 0 disables in-flight updates (a final run "
             "still happens on landing).",
    )
    parser.add_argument("--odm-feature-quality", default="medium",
                         choices=["ultra", "high", "medium", "low", "lowest"])
    parser.add_argument("--odm-orthophoto-resolution", type=float, default=5.0,
                         help="Centimetres per pixel for the ODM orthophoto. Higher "
                              "= less RAM at the cost of ground detail.")
    parser.add_argument(
        "--no-realism", action="store_true",
        help="Save frames straight from the sim, skipping camera_realism.py's "
             "SIYI-A2-mini-like degradation pipeline (lens/noise/compression).",
    )
    parser.add_argument(
        "--run-odm-on", default=None, metavar="SESSION_DIR",
        help="Skip flying/capturing entirely - just (re)run ODM on an "
             "already-captured session dir, e.g. captures/2026-08-30_123809 "
             "(useful if the automatic run failed, e.g. Docker wasn't up yet).",
    )
    args = parser.parse_args()
    if args.mission_planner_udp_port is not None and args.lr900_usb_port is not None:
        parser.error("Choose either --mission-planner-udp-port or --lr900-usb-port, not both")
    if args.mission_planner_udp_port is not None:
        try:
            args.connection = mission_planner_udp_connection(args.mission_planner_host,
                                                             args.mission_planner_udp_port)
        except ValueError as exc:
            parser.error(str(exc))
    if args.lr900_usb_port is not None:
        try:
            args.connection, args.lr900_baud = lr900_usb_connection(args.lr900_usb_port,
                                                                      args.lr900_baud)
        except ValueError as exc:
            parser.error(str(exc))
    if args.rtsp_url is not None:
        try:
            args.rtsp_url = validate_rtsp_url(args.rtsp_url)
        except ValueError as exc:
            parser.error(str(exc))
    if not math.isfinite(args.capture_interval_m) or args.capture_interval_m <= 0:
        parser.error("--capture-interval-m must be positive and finite")
    if not math.isfinite(args.min_altitude_m) or args.min_altitude_m < 1:
        parser.error("--min-altitude-m must be finite and at least 1 m")
    if not math.isfinite(args.camera_hfov_deg) or not 0 < args.camera_hfov_deg < 180:
        parser.error("--camera-hfov-deg must be finite and between 0 and 180")
    if not math.isfinite(args.rtsp_startup_timeout) or args.rtsp_startup_timeout <= 0:
        parser.error("--rtsp-startup-timeout must be positive and finite")
    camera_make = args.camera_make or ("SIYI" if args.rtsp_url is not None else CAMERA_MAKE)
    camera_model = args.camera_model or ("RTSP-main.264" if args.rtsp_url is not None else CAMERA_MODEL)
    focal_length_35mm = round(18.0 / math.tan(math.radians(args.camera_hfov_deg) / 2))

    if args.run_odm_on:
        run_odm(Path(args.run_odm_on), args.odm_feature_quality, args.odm_orthophoto_resolution)
        return

    if not args.polygon:
        sys.exit("--polygon is required: a JSON file of [lat, lon] vertices for the area to map.")
    polygon = GpsPolygon.load(args.polygon)
    print(f"Mapping area: {len(polygon.vertices)}-vertex polygon centered near "
          f"({polygon.ref_lat:.6f}, {polygon.ref_lon:.6f}).")

    topic = build_camera_topic(args.world, args.model)

    if args.mission_planner_udp_port is not None:
        print(f"Receiving MAVLink forwarded by Mission Planner on {args.connection} ...")
    elif args.lr900_usb_port is not None:
        print(f"Reading MAVLink from LR900 USB serial port {args.connection} at {args.lr900_baud} Bd ...")
    else:
        print(f"Connecting to {args.connection} ...")
    mav = mavutil.mavlink_connection(args.connection, **(
        {"baud": args.lr900_baud} if args.lr900_usb_port is not None else {}))
    if mav.wait_heartbeat(timeout=10) is None:
        sys.exit(f"No MAVLink heartbeat on '{args.connection}' - is the SITL sim running?")
    print("Connected.")

    gimbal = None
    telemetry_only = args.mission_planner_udp_port is not None or args.lr900_usb_port is not None
    if not telemetry_only or args.relay_gimbal_control:
        print("Pointing gimbal down ...")
        gimbal = GimbalController(connection=mav)
        gimbal.point_down()
    else:
        print("External telemetry mode: gimbal control is disabled; point the camera down before flight.")

    if args.rtsp_url is not None:
        print(f"Opening RTSP camera stream: {args.rtsp_url}")
        if args.camera_hfov_deg == CAMERA_HFOV_DEG:
            print("WARNING: --camera-hfov-deg is still the simulator default (114.6). "
                  "Set it to the real camera's wide-view horizontal FOV for an accurate map.")
        stream = RtspCameraStream(args.rtsp_url)
        if not stream.wait_for_frame(args.rtsp_startup_timeout):
            stream.close()
            mav.close()
            sys.exit(f"No RTSP frame received within {args.rtsp_startup_timeout:g} s: {args.rtsp_url}")
        print("RTSP camera stream is delivering frames.")
    else:
        if GzCameraStream is None:
            mav.close()
            sys.exit("Gazebo Transport is unavailable. Supply --rtsp-url for a real camera stream.")
        print(f"Subscribing to camera topic: {topic}")
        stream = GzCameraStream(topic=topic)

    session_name = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    session_dir = Path(args.output_dir) / session_name
    images_dir = session_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)
    mosaic_images_dir = session_dir / "mosaic_images"
    mosaic_images_dir.mkdir(parents=True, exist_ok=True)
    (session_dir / "capture_config.json").write_text(json.dumps({
        "image_source": args.rtsp_url or "gazebo",
        "telemetry_source": (f"lr900-usb:{args.connection}" if args.lr900_usb_port else
                             f"mission-planner-udp:{args.connection}" if args.mission_planner_udp_port else
                             args.connection),
        "camera_hfov_deg": args.camera_hfov_deg,
        "camera_make": camera_make,
        "camera_model": camera_model,
        "cropped_width_fraction": 1.0 if args.rtsp_url is not None else 1.0 - 2*AIRFRAME_CROP_FRACTION,
        "synthetic_camera_realism": not args.no_realism and args.rtsp_url is None,
    }, indent=2) + "\n")
    print(f"Saving captures to: {images_dir}")

    mosaic_path = Path(args.mosaic_out) if args.mosaic_out else session_dir / "live_map.jpg"
    cropped_width_fraction = 1.0 if args.rtsp_url is not None else 1.0 - 2 * AIRFRAME_CROP_FRACTION
    mosaic = LiveMosaic(
        polygon, camera_hfov_deg=args.camera_hfov_deg,
        cropped_width_fraction=cropped_width_fraction,
        pixels_per_meter=args.mosaic_resolution,
    )
    print(f"Live map will be kept up to date at: {mosaic_path}")

    armed = False
    armed_at = None
    in_area = False
    last_lat: Optional[float] = None
    last_lon: Optional[float] = None
    roll_deg = 0.0
    pitch_deg = 0.0
    image_count = 0
    last_odm_trigger_count = 0
    odm_runner = OdmRunner(args.odm_feature_quality, args.odm_orthophoto_resolution) if args.odm else None
    inflight = (InflightMatcher(session_dir, fast=args.final_map_fast)
                if not args.no_final_map and not args.no_inflight_matching else None)

    print(f"Watching telemetry (capture every {args.capture_interval_m} m once armed "
          "and inside the mapping area) ...")
    try:
        while True:
            msg = mav.recv_match(
                type=["HEARTBEAT", "GLOBAL_POSITION_INT", "ATTITUDE"], blocking=True, timeout=1.0
            )
            if msg is None:
                continue

            if msg.get_type() == "ATTITUDE":
                roll_deg = math.degrees(msg.roll)
                pitch_deg = math.degrees(msg.pitch)
                continue

            if msg.get_type() == "HEARTBEAT":
                if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                    continue
                now_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if now_armed and not armed:
                    armed_at = time.monotonic()
                if armed and not now_armed:
                    disarmed_at = time.monotonic()
                    print(f"\nDisarmed. {image_count} image(s) captured.")
                    mosaic.save(mosaic_path)
                    print(f"Live map: {mosaic_path}")
                    if inflight is not None:
                        inflight.close()
                    if not args.no_final_map and image_count >= MIN_IMAGES_FOR_FINAL_MAP:
                        print("Aligning captured frames for final_map.png ...", flush=True)
                        result = subprocess.run([
                            sys.executable, str(Path(__file__).with_name("rebuild_mosaic.py")),
                            str(session_dir), "--polygon", args.polygon, "--align",
                            "--resolution", str(args.mosaic_resolution),
                            "--hfov", str(args.camera_hfov_deg),
                            "--cropped-width-fraction", str(cropped_width_fraction),
                            "--out", str(session_dir / "final_map.png"),
                            *(["--fast"] if args.final_map_fast else []),
                        ], check=False)
                        if result.returncode:
                            print("Final alignment failed; the live map and source images are still available.")
                        elapsed = time.monotonic() - armed_at
                        timing = {"armed_to_disarmed_seconds": disarmed_at-armed_at,
                                  "armed_to_map_attempt_end_seconds": elapsed,
                                  "map_succeeded": result.returncode == 0,
                                  "target_seconds": 720,
                                  "within_12_minutes": result.returncode == 0 and elapsed <= 720,
                                  "timing_origin": "first observed armed heartbeat; start capture before arming"}
                        (session_dir / "mission_timing.json").write_text(json.dumps(timing, indent=2)+"\n")
                        print(f"Time from observed arming: {elapsed/60:.2f} min; "
                              f"12-minute target met: {timing['within_12_minutes']}", flush=True)
                    if args.odm:
                        if image_count >= MIN_IMAGES_FOR_ODM:
                            if odm_runner is not None:
                                odm_runner.wait_idle()
                            run_odm(session_dir, args.odm_feature_quality, args.odm_orthophoto_resolution, label="final")
                        else:
                            print("Too few images for a meaningful ODM reconstruction - skipping.")
                    break
                armed = now_armed
                continue

            # GLOBAL_POSITION_INT
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt_m = msg.relative_alt / 1000.0
            heading_deg = msg.hdg / 100.0 if msg.hdg != 65535 else 0.0

            if not armed or alt_m < args.min_altitude_m:
                continue

            now_in_area = polygon.contains(lat, lon)
            if now_in_area != in_area:
                in_area = now_in_area
                verb = "Entered" if in_area else "Left"
                print(f"\n{verb} mapping area @ ({lat:.6f}, {lon:.6f}).")
            if not in_area:
                continue

            if last_lat is not None:
                dist = _planar_distance_m(last_lat, last_lon, lat, lon)
                if dist < args.capture_interval_m:
                    continue

            if max(abs(roll_deg), abs(pitch_deg)) > args.max_tilt_deg:
                continue

            frame = stream.latest()
            if frame is None:
                continue
            # Real RTSP frames must stay intact: the crop and synthetic
            # degradation are specific to the simulated x500 camera.
            mosaic_frame = frame if args.rtsp_url is not None else _crop_airframe(frame)
            if not args.no_realism and args.rtsp_url is None:
                frame = apply_camera_realism(frame)
            frame = frame if args.rtsp_url is not None else _crop_airframe(frame)

            image_count += 1
            image_path = images_dir / f"IMG_{image_count:04d}.jpg"
            _write_geotagged_jpeg(frame, image_path, lat, lon, alt_m, heading_deg,
                                  camera_make, camera_model, focal_length_35mm)
            # Preserve the same clean view used by the live map for final
            # alignment, without synthetic lens distortion/noise added for ODM.
            _write_geotagged_jpeg(mosaic_frame, mosaic_images_dir / image_path.name,
                                 lat, lon, alt_m, heading_deg,
                                 camera_make, camera_model, focal_length_35mm)
            if inflight is not None:
                try:
                    inflight.submit(mosaic_images_dir / image_path.name, lat, lon, alt_m)
                except (OSError, RuntimeError) as exc:
                    print(f"\nIn-flight matching unavailable: {exc}; capture continues, final map will fill missing work.", flush=True)
                    inflight.close()
                    inflight = None
            mosaic.update(mosaic_frame, lat, lon, alt_m, heading_deg)
            mosaic.save(mosaic_path)
            last_lat, last_lon = lat, lon
            print(f"\rCaptured {image_path.name} @ ({lat:.6f}, {lon:.6f}, {alt_m:.1f}m)", end="", flush=True)

            if (odm_runner is not None and args.odm_update_interval > 0
                    and image_count - last_odm_trigger_count >= args.odm_update_interval):
                started = odm_runner.trigger(session_dir, label=f"in-flight, {image_count} images")
                if started:
                    last_odm_trigger_count = image_count
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if inflight is not None:
            inflight.close()
        if gimbal is not None:
            gimbal.close()
        if hasattr(stream, "close"):
            stream.close()
        mav.close()


if __name__ == "__main__":
    main()
