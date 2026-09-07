"""Rebuild a GPS mosaic from captured JPEGs without Gazebo or MAVLink."""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import piexif

from area import GpsPolygon
from live_mosaic import LiveMosaic


def ratio(value):
    return value[0] / value[1]


def coordinate(value, ref):
    result = sum(ratio(v) / scale for v, scale in zip(value, (1, 60, 3600)))
    return -result if ref in (b"S", b"W") else result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--polygon", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--estimate-heading", action="store_true",
                        help="Allow approximate course-derived headings for old sessions without EXIF heading.")
    parser.add_argument("--resolution", type=float, default=8.0)
    parser.add_argument("--align", action="store_true", help="Globally align matching image features before compositing")
    parser.add_argument("--hfov", type=float, default=114.6)
    parser.add_argument("--cropped-width-fraction", type=float, default=0.8)
    args = parser.parse_args()
    records = []
    images_dir = args.session / "mosaic_images"
    if not images_dir.is_dir():
        images_dir = args.session / "images"
    print(f"Reading {images_dir}", flush=True)
    for path in sorted(images_dir.glob("*.jpg")):
        gps = piexif.load(str(path))["GPS"]
        lat = coordinate(gps[2], gps[1])
        lon = coordinate(gps[4], gps[3])
        # These capture files store relative height in GPSAltitude.
        altitude = ratio(gps[6]) * (-1 if gps.get(5, 0) == 1 else 1)
        if altitude < 1.0:
            print(f"Skipping {path.name}: height {altitude:.2f} m (on the ground)")
            continue
        heading = ratio(gps[17]) if 17 in gps else None
        if heading is not None and gps.get(16, b"T") != b"T":
            parser.error(f"{path}: heading must reference true north")
        records.append((path, lat, lon, altitude, heading))
    if not records:
        parser.error("No JPEGs found in session/images")
    missing = sum(r[4] is None for r in records)
    if missing and not args.estimate_heading:
        parser.error("Missing EXIF headings; --estimate-heading explicitly enables an approximate reconstruction")
    if missing and len(records) < 2:
        parser.error("At least two positions are needed to estimate heading")
    if missing:
        print(f"WARNING: estimating {missing} camera headings from travel direction; turns and sideslip may misalign.")
    mosaic = LiveMosaic(GpsPolygon.load(args.polygon), args.hfov,
                        cropped_width_fraction=args.cropped_width_fraction,
                        pixels_per_meter=args.resolution)
    for i, (path, lat, lon, altitude, heading) in enumerate(records):
        if heading is None:
            a, b = (records[i], records[i + 1]) if i + 1 < len(records) else (records[i - 1], records[i])
            north = b[1] - a[1]
            east = (b[2] - a[2]) * math.cos(math.radians(lat))
            if math.hypot(east, north) < 1e-9:
                parser.error(f"{path}: cannot estimate heading from stationary positions")
            heading = math.degrees(math.atan2(east, north)) % 360
        records[i] = (path, lat, lon, altitude, heading)
    poses = shapes = usable = None
    if args.align:
        from align_mosaic import align_frames
        poses, shapes, usable = align_frames(records, mosaic._area, args.hfov, args.cropped_width_fraction)
    for i, (path, lat, lon, altitude, heading) in enumerate(records):
        if usable is not None and not usable[i]:
            continue
        frame = cv2.imread(str(path))
        if frame is None:
            parser.error(f"Unreadable image: {path}")
        if poses is None:
            mosaic.update(frame, lat, lon, altitude, heading)
        else:
            a, b, east, south = poses[i]
            h, w = shapes[i]
            linear = np.array([[a, -b], [b, a]]) * args.resolution / (w / 2)
            center = np.array([east - mosaic._min_e, south + mosaic._max_n]) * args.resolution
            matrix = np.column_stack((linear, center - linear @ np.array([w / 2, h / 2])))
            mosaic.update_projected(frame, matrix)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    mosaic.save(args.out)
    used = len(records) if usable is None else int(usable.sum())
    print(f"Saved {args.out} ({used}/{len(records)} frames used)")


if __name__ == "__main__":
    main()
