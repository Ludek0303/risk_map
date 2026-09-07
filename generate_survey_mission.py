"""Generate a QGroundControl .plan lawnmower survey mission.

Builds a boustrophedon (back-and-forth) grid of MAV_CMD_NAV_WAYPOINT items
covering a rectangular (or square) area centered on a given lat/lon,
bracketed by a takeoff and a landing back at the home position (so the
vehicle lands and disarms on its own at the end - which is what
mapping_capture.py's ODM auto-trigger is watching for). Uses MAV_CMD_NAV_LAND
rather than RETURN_TO_LAUNCH: this PX4 build rejects RTL as an in-mission
item ("Command is not supported") since PX4 only implements RTL as a
flight-mode switch, not a mission item.

Line spacing is derived from the x500_gimbal camera's actual horizontal FOV
(2.0 rad / 114.6 deg, from PX4-Autopilot's gimbal/model.sdf) so the flight
lines overlap enough for OpenDroneMap to find matching features between
adjacent passes. Flight lines always run north-south (spaced out east-west);
give the north-south extent as --height-m and the east-west extent as
--width-m if the area isn't square.

Usage:
    python3 generate_survey_mission.py --area-m2 40000 --out missions/survey.plan

    # or a specific (possibly non-square) rectangle:
    python3 generate_survey_mission.py --width-m 147 --height-m 262 \\
        --altitude-m 45 --out missions/survey.plan

    # centers on the vehicle's current position by default (reads it live
    # over MAVLink); pass --center-lat/--center-lon to use fixed coordinates
    # instead (e.g. to plan a mission before the sim is even running).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Optional

from pymavlink import mavutil

from gimbal_control import DEFAULT_CONNECTION

MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_LAND = 21
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

CAMERA_HFOV_DEG = 114.6  # x500_gimbal cgo3 camera, from model.sdf <horizontal_fov>
DEFAULT_SIDELAP = 0.7  # fraction of footprint width shared between adjacent lines


def get_current_position(connection: str) -> tuple[float, float, float]:
    print(f"Connecting to {connection} to read current position ...")
    mav = mavutil.mavlink_connection(connection)
    if mav.wait_heartbeat(timeout=10) is None:
        sys.exit(f"No MAVLink heartbeat on '{connection}' - is the SITL sim running? "
                  "(or pass --center-lat/--center-lon to skip this)")
    msg = mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    if msg is None:
        sys.exit("No GLOBAL_POSITION_INT received.")
    home = mav.recv_match(type="HOME_POSITION", blocking=True, timeout=3)
    home_alt_amsl = home.altitude / 1000.0 if home is not None else msg.alt / 1000.0
    mav.close()
    return msg.lat / 1e7, msg.lon / 1e7, home_alt_amsl


def meters_to_latlon_offset(lat0: float, dx_east: float, dy_north: float) -> tuple[float, float]:
    dlat = dy_north / 111320.0
    dlon = dx_east / (111320.0 * math.cos(math.radians(lat0)))
    return dlat, dlon


def build_grid_lines(width_m: float, height_m: float, spacing_m: float) -> list[list[tuple[float, float]]]:
    """Returns a list of lines, each a list of (east_m, north_m) offsets from
    the area center, already ordered boustrophedon (alternating direction).
    Lines run north-south (height_m long), spaced out east-west (width_m)."""
    half_x = width_m / 2.0
    half_y = height_m / 2.0
    n_lines = max(2, math.ceil(width_m / spacing_m) + 1)
    xs = [-half_x + i * (width_m / (n_lines - 1)) for i in range(n_lines)]

    lines = []
    for i, x in enumerate(xs):
        y_start, y_end = (-half_y, half_y) if i % 2 == 0 else (half_y, -half_y)
        lines.append([(x, y_start), (x, y_end)])
    return lines


def build_plan(
    center_lat: float, center_lon: float, home_alt_amsl: float,
    width_m: float, height_m: float, altitude_m: float, spacing_m: float,
    cruise_speed: float,
) -> dict:
    lines = build_grid_lines(width_m, height_m, spacing_m)

    items = []
    do_jump_id = 1

    first_east, first_north = lines[0][0]
    dlat, dlon = meters_to_latlon_offset(center_lat, first_east, first_north)
    items.append({
        "AMSLAltAboveTerrain": None,
        "Altitude": altitude_m,
        "AltitudeMode": 1,
        "autoContinue": True,
        "command": MAV_CMD_NAV_TAKEOFF,
        "doJumpId": do_jump_id,
        "frame": MAV_FRAME_GLOBAL_RELATIVE_ALT,
        "params": [0, 0, 0, None, center_lat + dlat, center_lon + dlon, altitude_m],
        "type": "SimpleItem",
    })
    do_jump_id += 1

    for line in lines:
        for east_m, north_m in line:
            dlat, dlon = meters_to_latlon_offset(center_lat, east_m, north_m)
            items.append({
                "AMSLAltAboveTerrain": None,
                "Altitude": altitude_m,
                "AltitudeMode": 1,
                "autoContinue": True,
                "command": MAV_CMD_NAV_WAYPOINT,
                "doJumpId": do_jump_id,
                "frame": MAV_FRAME_GLOBAL_RELATIVE_ALT,
                "params": [0, 0, 0, None, center_lat + dlat, center_lon + dlon, altitude_m],
                "type": "SimpleItem",
            })
            do_jump_id += 1

    items.append({
        "AMSLAltAboveTerrain": None,
        "Altitude": 0,
        "AltitudeMode": 1,
        "autoContinue": True,
        "command": MAV_CMD_NAV_LAND,
        "doJumpId": do_jump_id,
        "frame": MAV_FRAME_GLOBAL_RELATIVE_ALT,
        "params": [0, 0, 0, None, center_lat, center_lon, 0],
        "type": "SimpleItem",
    })

    return {
        "fileType": "Plan",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": cruise_speed,
            "firmwareType": 12,  # MAV_AUTOPILOT_PX4
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": 5,
            "items": items,
            "plannedHomePosition": [center_lat, center_lon, home_alt_amsl],
            "vehicleType": 2,  # MultiRotor
            "version": 2,
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--area-m2", type=float, default=40000.0,
                         help="Total area to cover, as a square (default 40000 = 200x200m). "
                              "Ignored if --width-m/--height-m are given.")
    parser.add_argument("--width-m", type=float, default=None,
                         help="East-west extent of a rectangular area to cover. "
                              "Must be given together with --height-m; overrides --area-m2.")
    parser.add_argument("--height-m", type=float, default=None,
                         help="North-south extent of a rectangular area to cover "
                              "(length of each flight line). Must be given together "
                              "with --width-m; overrides --area-m2.")
    parser.add_argument("--altitude-m", type=float, default=45.0)
    parser.add_argument("--sidelap", type=float, default=DEFAULT_SIDELAP,
                         help="Fraction of camera footprint width shared between adjacent lines (0-1).")
    parser.add_argument("--cruise-speed", type=float, default=8.0)
    parser.add_argument("--center-lat", type=float, default=None)
    parser.add_argument("--center-lon", type=float, default=None)
    parser.add_argument("--home-alt-amsl", type=float, default=None)
    parser.add_argument("--connection", default=DEFAULT_CONNECTION)
    parser.add_argument("--out", default="missions/survey.plan")
    args = parser.parse_args()

    if (args.width_m is None) != (args.height_m is None):
        parser.error("--width-m and --height-m must be given together.")

    if args.center_lat is None or args.center_lon is None:
        lat, lon, home_alt = get_current_position(args.connection)
    else:
        lat, lon = args.center_lat, args.center_lon
        home_alt = args.home_alt_amsl if args.home_alt_amsl is not None else 0.0

    if args.width_m is not None:
        width_m, height_m = args.width_m, args.height_m
    else:
        width_m = height_m = math.sqrt(args.area_m2)

    footprint_w = 2 * args.altitude_m * math.tan(math.radians(CAMERA_HFOV_DEG) / 2)
    spacing_m = footprint_w * (1 - args.sidelap)

    print(f"Center: {lat:.6f}, {lon:.6f} (home alt AMSL: {home_alt:.1f}m)")
    print(f"Area: {width_m:.1f} x {height_m:.1f} m (width x height)")
    print(f"Altitude: {args.altitude_m} m -> camera footprint width ~{footprint_w:.1f} m")
    print(f"Line spacing: {spacing_m:.1f} m ({args.sidelap*100:.0f}% sidelap)")

    plan = build_plan(lat, lon, home_alt, width_m, height_m, args.altitude_m, spacing_m, args.cruise_speed)

    n_waypoints = len(plan["mission"]["items"]) - 2  # minus takeoff/RTL
    n_lines = n_waypoints // 2
    path_len_m = n_lines * height_m  # each line is one height_m leg
    print(f"{n_waypoints} waypoints, ~{path_len_m:.0f} m flight path, "
          f"~{path_len_m/args.cruise_speed/60:.1f} min at {args.cruise_speed} m/s cruise")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(plan, indent=4))
    print(f"\nWrote {out_path}")
    print("In QGC: Plan view -> File -> Open -> select this file -> Upload (the arrow icon).")


if __name__ == "__main__":
    main()
