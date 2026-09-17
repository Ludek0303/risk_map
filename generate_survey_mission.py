"""Generate a Mission Planner .waypoints or QGroundControl .plan survey mission.

Mission Planner (ArduCopter) is the default output; --out FILE.plan keeps PX4/QGC.
For four GPS corners and initial laps use --mission-config FILE.json.
Use --lap-altitude-m for loop height and --altitude-m for survey height.

Builds a boustrophedon (back-and-forth) grid of MAV_CMD_NAV_WAYPOINT items
covering a rectangular (or square) area centered on a given lat/lon,
bracketed by a takeoff and a landing back at the home position (so the
vehicle lands and disarms on its own at the end - which is what
mapping_capture.py's ODM auto-trigger is watching for). Uses MAV_CMD_NAV_LAND
rather than RETURN_TO_LAUNCH: this PX4 build rejects RTL as an in-mission
item ("Command is not supported") since PX4 only implements RTL as a
flight-mode switch, not a mission item.

Line spacing uses the shorter ground footprint of the cropped x500_gimbal
camera (114.6 degree horizontal FOV, 1280x720 source, 80% retained width).
Defaults target 80% sidelap, at least 90% frontlap and 3 m/s groundspeed.
Endpoints are inset 2 m and include settling stops. The companion capture
script points the gimbal down and saves images; this mission does not itself
trigger the Gazebo camera. Also writes an area polygon and planning report.
Flight lines run north-south, spaced east-west.
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

from mission_geometry import load_config, polygon_grid, lap_route
from mission_planner_export import mission_planner_text, mission_planner_fence_text

DEFAULT_CONNECTION = "udpin:127.0.0.1:14540"

MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_LAND = 21
MAV_CMD_DO_JUMP = 177
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

CAMERA_HFOV_DEG = 114.6  # x500_gimbal cgo3 camera, from model.sdf <horizontal_fov>
DEFAULT_SIDELAP = 0.8  # fraction of footprint width shared between adjacent lines
FT_TO_M = 0.3048


def get_current_position(connection: str) -> tuple[float, float, float]:
    from pymavlink import mavutil

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


def start_survey_lines_near(lines: list, target: tuple[float, float]) -> list:
    """Re-orient a boustrophedon (back-and-forth) grid so its first point is
    the closest of its 4 possible entry corners to `target`, without changing
    the lines themselves - only which end of the zigzag is flown first.
    Reversing every individual line (mirror) flips which side (near/far end
    of each line) comes first while keeping the same column order; reversing
    the whole list (reverse) flies the same zigzag from the other column end.
    Combining both gives all 4 corners."""
    best = lines
    best_distance = math.inf
    for mirror in (False, True):
        for reverse in (False, True):
            candidate = [line[::-1] for line in lines] if mirror else list(lines)
            if reverse:
                candidate = list(reversed(candidate))
            distance = math.dist(candidate[0][0], target)
            if distance < best_distance:
                best, best_distance = candidate, distance
    return best


def meters_to_latlon_offset(lat0: float, dx_east: float, dy_north: float) -> tuple[float, float]:
    dlat = dy_north / 111320.0
    dlon = dx_east / (111320.0 * math.cos(math.radians(lat0)))
    return dlat, dlon


def build_grid_lines(width_m: float, height_m: float, spacing_m: float) -> list[list[tuple[float, float]]]:
    """Returns a list of lines, each a list of (east_m, north_m) offsets from
    the area center, already ordered boustrophedon (alternating direction).
    Lines run north-south (height_m long), spaced out east-west (width_m)."""
    if not all(math.isfinite(v) and v > 0 for v in (width_m, height_m, spacing_m)):
        raise ValueError("Grid dimensions and spacing must be positive and finite")
    # Keep centers inside the capture polygon, away from its strict boundary.
    half_x = width_m / 2.0 - min(2.0, width_m/4)
    half_y = height_m / 2.0 - min(2.0, height_m/4)
    n_lines = max(2, math.ceil(2*half_x / spacing_m) + 1)
    xs = [-half_x + i * (2*half_x / (n_lines - 1)) for i in range(n_lines)]

    lines = []
    for i, x in enumerate(xs):
        y_start, y_end = (-half_y, half_y) if i % 2 == 0 else (half_y, -half_y)
        lines.append([(x, y_start), (x, y_end)])
    return lines


def build_plan(
    center_lat: float, center_lon: float, home_alt_amsl: float,
    width_m: float, height_m: float, altitude_m: float, spacing_m: float,
    cruise_speed: float, settle_seconds: float = 3.0,
    *, survey_lines=None, prefix_points=None, laps=1, lap_altitude_m=None, lap_speed=None, transfer_speed=None,
    lap_delay_s=None, riskmapping_delay_s=None,
) -> dict:
    if not all(math.isfinite(v) and v > 0 for v in
               (width_m, height_m, altitude_m, spacing_m, cruise_speed)):
        raise ValueError("Dimensions, altitude, spacing and speed must be positive and finite")
    if not math.isfinite(settle_seconds) or settle_seconds < 0:
        raise ValueError("Settling time must be finite and non-negative")
    lap_delay_s = settle_seconds if lap_delay_s is None else lap_delay_s
    riskmapping_delay_s = settle_seconds if riskmapping_delay_s is None else riskmapping_delay_s
    if any(not math.isfinite(v) or v < 0 for v in (lap_delay_s, riskmapping_delay_s)):
        raise ValueError("Phase delays must be finite and non-negative")
    if not (-89 < center_lat < 89 and -180 <= center_lon <= 180
            and math.isfinite(home_alt_amsl)):
        raise ValueError("Invalid home coordinates (polar surveys are unsupported)")
    if lap_altitude_m is None:
        lap_altitude_m = altitude_m
    if not math.isfinite(lap_altitude_m) or lap_altitude_m <= 0:
        raise ValueError("Lap altitude must be positive and finite")
    if lap_speed is None:
        lap_speed = cruise_speed
    if not math.isfinite(lap_speed) or lap_speed <= 0:
        raise ValueError("Lap speed must be positive and finite")
    if transfer_speed is None:
        transfer_speed = cruise_speed
    if not math.isfinite(transfer_speed) or transfer_speed <= 0:
        raise ValueError("Transfer speed must be positive and finite")
    current_speed = lap_speed if prefix_points else cruise_speed
    takeoff_altitude = lap_altitude_m if prefix_points else altitude_m
    lines = (build_grid_lines(width_m, height_m, spacing_m)
             if survey_lines is None else survey_lines)

    items = []
    do_jump_id = 1

    # Climb vertically over home before travelling to the first survey line.
    dlat, dlon = 0.0, 0.0
    items.append({
        "AMSLAltAboveTerrain": None,
        "Altitude": takeoff_altitude,
        "AltitudeMode": 1,
        "autoContinue": True,
        "command": MAV_CMD_NAV_TAKEOFF,
        "doJumpId": do_jump_id,
        "frame": MAV_FRAME_GLOBAL_RELATIVE_ALT,
        "params": [0, 0, 0, None, center_lat + dlat, center_lon + dlon, takeoff_altitude],
        "type": "SimpleItem",
    })
    do_jump_id += 1

    # QGC cruiseSpeed is metadata; command the actual multirotor groundspeed.
    items.append({"autoContinue": True, "command": 178, "doJumpId": do_jump_id,
                  "frame": 2, "params": [1, current_speed, -1, 0, 0, 0, 0],
                  "type": "SimpleItem"})
    do_jump_id += 1

    # Return horizontally at survey altitude, then descend vertically at home.
    segments = [(prefix_points or [], lap_altitude_m, current_speed, lap_delay_s)]
    if prefix_points:
        # Transfer to the first mapping waypoint while still at loop altitude
        # and transfer speed, then change altitude in place (if different)
        # before travelling along the first photo line.
        segments.append(([lines[0][0]], lap_altitude_m, transfer_speed, riskmapping_delay_s))
        if lap_altitude_m != altitude_m:
            segments.append(([lines[0][0]], altitude_m, transfer_speed, riskmapping_delay_s))
        survey_segments = [lines[0][1:], *lines[1:]]
    else:
        survey_segments = lines
    segments.extend((line, altitude_m, cruise_speed, riskmapping_delay_s) for line in [*survey_segments, [(0.0, 0.0)]])
    for segment_index, (line, waypoint_altitude, segment_speed, waypoint_delay) in enumerate(segments):
        if line and segment_speed != current_speed:
            items.append({"autoContinue": True, "command": 178, "doJumpId": do_jump_id,
                          "frame": 2, "params": [1, segment_speed, -1, 0, 0, 0, 0],
                          "type": "SimpleItem"})
            do_jump_id += 1
            current_speed = segment_speed
        second_lap_point_row = None
        for point_index, (east_m, north_m) in enumerate(line):
            dlat, dlon = meters_to_latlon_offset(center_lat, east_m, north_m)
            items.append({
                "AMSLAltAboveTerrain": None,
                "Altitude": waypoint_altitude,
                "AltitudeMode": 1,
                "autoContinue": True,
                "command": MAV_CMD_NAV_WAYPOINT,
                "doJumpId": do_jump_id,
                "frame": MAV_FRAME_GLOBAL_RELATIVE_ALT,
                "params": [waypoint_delay, 1, 0, None, center_lat + dlat, center_lon + dlon, waypoint_altitude],
                "type": "SimpleItem",
            })
            do_jump_id += 1
            # Row numbers are 1-based (row 0 is home, prepended by the
            # exporter), so an item's WPL row equals its 1-based position
            # in `items` - i.e. len(items) right after appending it.
            if segment_index == 0 and point_index == 1:
                second_lap_point_row = len(items)
        if segment_index == 0 and laps > 1 and second_lap_point_row is not None:
            # Fly the loop once above, then jump back to its second point
            # (not the first, which only appears once, at the very start)
            # for the remaining laps - avoids writing every lap out in full.
            items.append({"autoContinue": True, "command": MAV_CMD_DO_JUMP, "doJumpId": do_jump_id,
                          "frame": 2, "params": [second_lap_point_row, laps - 1, 0, 0, 0, 0, 0],
                          "type": "SimpleItem"})
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
            # Mission Planner deserializes both metadata speeds as Int32.
            # Exact fractional groundspeeds remain in DO_CHANGE_SPEED items.
            "cruiseSpeed": max(1, round(cruise_speed)),
            "firmwareType": 12,  # MAV_AUTOPILOT_PX4
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": max(1, round(cruise_speed)),
            "items": items,
            "plannedHomePosition": [center_lat, center_lon, home_alt_amsl],
            "vehicleType": 2,  # MultiRotor
            "version": 2,
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1,
    }


def main(argv=None) -> None:
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
    parser.add_argument("--altitude-m", type=float, default=None,
                        help="Riskmapping altitude in m, relative to home (AGL); overrides JSON (default 45).")
    parser.add_argument("--altitude-ft", type=float, default=None,
                        help="Riskmapping altitude in ft, relative to home (AGL); alternative to --altitude-m.")
    parser.add_argument("--lap-altitude-m", type=float, default=None,
                        help="Takeoff and initial loop altitude relative to home (AGL), in m; defaults to --altitude-m.")
    parser.add_argument("--lap-altitude-ft", type=float, default=None,
                        help="Takeoff and initial loop altitude relative to home (AGL), in ft; alternative to --lap-altitude-m.")
    parser.add_argument("--sidelap", type=float, default=DEFAULT_SIDELAP,
                         help="Fraction of camera footprint width shared between adjacent lines (0-1).")
    parser.add_argument("--cruise-speed", type=float, default=None,
                        help="Riskmapping and return groundspeed in m/s; overrides JSON (default 3).")
    parser.add_argument("--lap-speed", type=float, default=None,
                        help="Groundspeed in m/s for initial loops and approach to them; defaults to --cruise-speed.")
    parser.add_argument("--transfer-speed", type=float, default=None,
                        help="Groundspeed between loops and riskmapping in m/s; overrides JSON.")
    parser.add_argument("--frontlap", type=float, default=0.9)
    parser.add_argument("--capture-interval-m", type=float, default=5.0,
                        help="Maximum photo spacing; reduced if frontlap requires it.")
    parser.add_argument("--settle-seconds", type=float, default=None,
                        help="Override both JSON phase delays, in seconds (fallback default 3).")
    parser.add_argument("--horizontal-accel-m-s2", type=float, default=None,
                        help="Optional acceleration assumption for flight-time estimation only; does not configure the autopilot.")
    parser.add_argument("--center-lat", type=float, default=None)
    parser.add_argument("--center-lon", type=float, default=None)
    parser.add_argument("--home-alt-amsl", type=float, default=None, help="Home elevation in m MSL.")
    parser.add_argument("--home-alt-amsl-ft", type=float, default=None,
                        help="Home elevation in ft MSL; alternative to --home-alt-amsl.")
    parser.add_argument("--connection", default=DEFAULT_CONNECTION)
    parser.add_argument("--format", choices=("mission-planner", "qgc"), default=None,
                        help="Output format; inferred from .waypoints/.plan, defaults to mission-planner.")
    parser.add_argument("--out", default=None)
    parser.add_argument("--mission-config", metavar="FILE.json",
                        help="JSON with area_points (four GPS corners), lap_points, laps; home is read from the drone GPS.")
    args = parser.parse_args(argv)
    for m_name, ft_name in (("altitude_m", "altitude_ft"), ("lap_altitude_m", "lap_altitude_ft"),
                            ("home_alt_amsl", "home_alt_amsl_ft")):
        ft_value = getattr(args, ft_name)
        if ft_value is not None:
            if getattr(args, m_name) is not None:
                parser.error(f"--{m_name.replace('_', '-')} and --{ft_name.replace('_', '-')} "
                             "are mutually exclusive")
            setattr(args, m_name, ft_value * FT_TO_M)
    suffix = Path(args.out).suffix.lower() if args.out else None
    inferred_format = {".waypoints": "mission-planner", ".plan": "qgc"}.get(suffix)
    if args.out and inferred_format is None:
        parser.error("--out must end in .waypoints (Mission Planner) or .plan (QGC)")
    if args.format and inferred_format and args.format != inferred_format:
        parser.error("--format does not match --out extension")
    args.format = args.format or inferred_format or "mission-planner"
    args.out = args.out or ("missions/survey.waypoints" if args.format == "mission-planner"
                            else "missions/survey.plan")
    config = None
    if args.mission_config:
        if args.width_m is not None or args.height_m is not None:
            parser.error("--mission-config cannot be combined with --width-m/--height-m")
        try:
            config = load_config(args.mission_config)
            polygon_grid(config[0], *config[0][0], 20.0)
        except (ValueError, OSError) as exc:
            parser.error(str(exc))


    if (args.width_m is None) != (args.height_m is None):
        parser.error("--width-m and --height-m must be given together.")

    if (args.center_lat is None) != (args.center_lon is None):
        parser.error("--center-lat and --center-lon must be given together.")
    settings = config[4] if config else {}
    for argument, key in (("altitude_m", "riskmapping_altitude_m"),
                          ("lap_altitude_m", "lap_altitude_m"),
                          ("cruise_speed", "riskmapping_speed_m_s"),
                          ("lap_speed", "lap_speed_m_s"),
                          ("transfer_speed", "transfer_speed_m_s")):
        if getattr(args, argument) is None:
            setattr(args, argument, settings.get(key))
    if args.altitude_m is None:
        args.altitude_m = 45.0
    if args.cruise_speed is None:
        args.cruise_speed = 3.0
    if args.transfer_speed is None:
        args.transfer_speed = args.cruise_speed
    if args.lap_altitude_m is None:
        args.lap_altitude_m = args.altitude_m
    if args.lap_speed is None:
        args.lap_speed = args.cruise_speed
    for name in ("transfer_speed", "lap_speed", "lap_altitude_m", "area_m2", "altitude_m", "cruise_speed", "capture_interval_m"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"{name} must be positive and finite")
    for value in (args.width_m, args.height_m):
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error("Area dimensions must be positive and finite")
    if not 0 <= args.sidelap < 1 or not 0 <= args.frontlap < 1:
        parser.error("Overlap must be in [0, 1)")
    # The legacy CLI option deliberately overrides both JSON phase delays.
    for name in ("lap_delay_s", "riskmapping_delay_s"):
        value = args.settle_seconds if args.settle_seconds is not None else settings.get(name, 3.0)
        if not math.isfinite(value) or value < 0:
            parser.error(f"{name} must be non-negative and finite")
        if args.format == "mission-planner":
            if value > 65535:
                parser.error(f"{name}: ArduCopter waypoint delay cannot exceed 65535 seconds")
            if value != int(value):
                value = float(math.ceil(value))
                print(f"ArduCopter {name} rounded up to {value:g} seconds")
        setattr(args, name, value)
    args.settle_seconds = args.riskmapping_delay_s
    if args.horizontal_accel_m_s2 is not None and (
            not math.isfinite(args.horizontal_accel_m_s2) or args.horizontal_accel_m_s2 <= 0):
        parser.error("--horizontal-accel-m-s2 must be positive and finite")
    if args.center_lat is None or args.center_lon is None:
        lat, lon, home_alt = get_current_position(args.connection)
    else:
        lat, lon = args.center_lat, args.center_lon
        home_alt = args.home_alt_amsl if args.home_alt_amsl is not None else 0.0
        if config:
            print("Placeholder home (row 0 / geofence anchor only): "
                  f"{lat:.6f}, {lon:.6f}. Regenerate with a live MAVLink "
                  "connection to set the real home before flight.")

    if args.width_m is not None:
        width_m, height_m = args.width_m, args.height_m
    else:
        width_m = height_m = math.sqrt(args.area_m2)

    footprint_w = 2 * args.altitude_m * math.tan(math.radians(CAMERA_HFOV_DEG) / 2)
    # Camera source is 1280x720, with 10% cropped from each horizontal side.
    # Use the shorter footprint for both directions: also valid when yaw changes.
    usable_footprint = min(footprint_w * 0.8, footprint_w * 720 / 1280)
    spacing_m = usable_footprint * (1 - args.sidelap)
    capture_interval = min(args.capture_interval_m, usable_footprint * (1-args.frontlap))

    print(f"Home: {lat:.6f}, {lon:.6f} (home alt AMSL: {home_alt:.1f}m)")
    if not config:
        print(f"Area: {width_m:.1f} x {height_m:.1f} m (width x height)")
    print(f"Altitude: {args.altitude_m} m -> camera footprint width ~{footprint_w:.1f} m")
    print(f"Line spacing: {spacing_m:.1f} m ({args.sidelap*100:.0f}% sidelap)")

    prefix = []
    lap_circuit = []
    laps = 0
    polygon = None
    lines = build_grid_lines(width_m, height_m, spacing_m)
    if config:
        polygon, lines = polygon_grid(config[0], lat, lon, spacing_m)
        # `prefix` is the fully unrolled route (every lap written out) used
        # below for the distance/time report; `lap_circuit` is just one pass
        # through the loop, flown `laps` times via MAV_CMD_DO_JUMP so the
        # mission file doesn't repeat every lap's waypoints.
        prefix = lap_route(config[1], config[2], lat, lon)
        laps = config[2]
        lap_circuit = prefix[:len(config[1]) + 1]
        if lap_circuit:
            # Start the mapping grid from whichever of its 4 entry corners is
            # closest to where the loop ends, instead of a fixed corner, so
            # the transfer leg out of the loop is as short as possible.
            lines = start_survey_lines_near(lines, lap_circuit[-1])
        print(f"Custom four-corner area; {laps} initial laps at {args.lap_altitude_m:g} m")
    try:
        plan = build_plan(lat, lon, home_alt, width_m, height_m, args.altitude_m,
                          spacing_m, args.cruise_speed, args.settle_seconds,
                          survey_lines=lines, prefix_points=lap_circuit, laps=laps or 1,
                          lap_altitude_m=args.lap_altitude_m,
                          lap_speed=args.lap_speed, transfer_speed=args.transfer_speed,
                          lap_delay_s=args.lap_delay_s, riskmapping_delay_s=args.riskmapping_delay_s)
    except ValueError as exc:
        parser.error(str(exc))

    boundary = config[3] if config else None
    if boundary:
        try:
            boundary.check_route([*polygon, polygon[0]], "Photo area")
            route = [item['params'][4:6] for item in plan['mission']['items']
                     if item['command'] in (16, 21, 22)]
            boundary.check_route(route, "Mission (including home)")
        except ValueError as exc:
            parser.error(str(exc))
        plan['geoFence']['polygons'] = [{"inclusion": True, "polygon": boundary.points, "version": 1}]

    points = [(0., 0.), *prefix, *(point for line in lines for point in line), (0., 0.)]
    distances = [math.dist(a, b) for a, b in zip(points, points[1:])]
    speeds = [args.lap_speed if i < len(prefix) else
              args.transfer_speed if prefix and i == len(prefix) else args.cruise_speed
              for i in range(len(distances))]
    path_len_m = sum(distances)
    horizontal_seconds = sum(distance/speed for distance, speed in zip(distances, speeds))
    print(f"Groundspeed: loops {args.lap_speed:g} m/s; transfer {args.transfer_speed:g} m/s; "
          f"survey and return {args.cruise_speed:g} m/s")
    transition = int(bool(prefix) and args.lap_altitude_m != args.altitude_m)
    vertical_distance = (args.lap_altitude_m + abs(args.lap_altitude_m-args.altitude_m)
                         + args.altitude_m) if prefix else 2*args.altitude_m
    # Estimate only: acceleration, wind and configured PX4 climb/descent rates vary.
    # The file only holds one physical pass through the loop (repeated laps
    # fly it again via MAV_CMD_DO_JUMP), so add back the delay from the laps
    # that aren't physically written out.
    total_delay_seconds = sum(item['params'][0] for item in plan['mission']['items']
                              if item['command'] == MAV_CMD_NAV_WAYPOINT)
    if laps > 1:
        total_delay_seconds += (laps - 1) * len(config[1]) * args.lap_delay_s
    flight_seconds = horizontal_seconds + total_delay_seconds + vertical_distance/1.5
    acceleration_seconds = 0.0
    if args.horizontal_accel_m_s2 is not None:
        acceleration = args.horizontal_accel_m_s2
        for distance, speed in zip(distances, speeds):
            # Symmetric acceleration/deceleration, with a full stop at each waypoint.
            leg_seconds = (2*math.sqrt(distance/acceleration) if distance < speed**2/acceleration
                           else distance/speed + speed/acceleration)
            acceleration_seconds += leg_seconds - distance/speed
        flight_seconds += acceleration_seconds
    actual_spacing = abs(lines[1][0][0] - lines[0][0][0])
    print(f"{len(lines)} lines, actual spacing {actual_spacing:.1f} m, "
          f"photos every {capture_interval:.1f} m")
    print(f"~{path_len_m:.0f} m horizontal route, estimated {flight_seconds/60:.1f} min "
          "including stops and assumed 1.5 m/s climb/descent")
    print(f"Acceleration allowance: {acceleration_seconds:.1f} s; "
          "actual duration depends on autopilot motion limits and flight conditions")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if args.format == "mission-planner":
        out_path.write_text(mission_planner_text(plan))
    else:
        out_path.write_text(json.dumps(plan, indent=4))
    fence_path = None
    if boundary and args.format == "mission-planner":
        fence_path = out_path.with_name(out_path.stem + "_fence.waypoints")
        fence_path.write_text(mission_planner_fence_text(plan, boundary))
    corners = [(-width_m/2, -height_m/2), (width_m/2, -height_m/2),
               (width_m/2, height_m/2), (-width_m/2, height_m/2)]
    if polygon is None:
        polygon = [[lat+dy, lon+dx] for east, north in corners
                   for dy, dx in [meters_to_latlon_offset(lat, east, north)]]
    area_path = out_path.with_name(out_path.stem + "_area.json")
    area_path.write_text(json.dumps(polygon, indent=2)+"\n")
    report = {"output_format": args.format,
              "flight_boundary": boundary.points if boundary else None,
              "fence_file": str(fence_path) if fence_path else None,
              "autopilot": "arducopter" if args.format == "mission-planner" else "px4",
              "settle_seconds": args.settle_seconds,  # legacy alias for riskmapping delay
              "lap_delay_s": args.lap_delay_s,
              "riskmapping_delay_s": args.riskmapping_delay_s,
              "total_delay_seconds": total_delay_seconds,
              "lines": len(lines), "line_spacing_m": actual_spacing,
              "conservative_footprint_m": usable_footprint,
              "sidelap_min": 1-actual_spacing/usable_footprint,
              "frontlap_min": 1-capture_interval/usable_footprint,
              "capture_interval_m": capture_interval, "speed_m_s": args.cruise_speed,
              "lap_speed_m_s": args.lap_speed,
              "riskmapping_speed_m_s": args.cruise_speed,
              "riskmapping_altitude_m": args.altitude_m,
              "transfer_speed_m_s": args.transfer_speed,
              "transfer_horizontal_distance_m": distances[len(prefix)] if prefix else 0.,
              "lap_horizontal_distance_m": sum(distances[:len(prefix)]),
              "estimated_horizontal_seconds": horizontal_seconds,
              "altitude_m": args.altitude_m, "horizontal_distance_m": path_len_m,
              "estimated_flight_seconds": flight_seconds,
              "assumed_horizontal_accel_m_s2": args.horizontal_accel_m_s2,
              "acceleration_allowance_seconds": acceleration_seconds,
              "laps": laps,
              "lap_waypoints": len(lap_circuit),
              "survey_start_item_id": (3 + len(lap_circuit) + int(laps > 1) + transition
                                       + int(bool(prefix) and args.lap_speed != args.transfer_speed)),
              "lap_altitude_m": args.lap_altitude_m,
              "vertical_distance_m": vertical_distance,
              "estimated_survey_photos": sum(math.floor(math.dist(*line)/capture_interval)+1 for line in lines),
              # AGL here means relative to home, as commanded to the autopilot;
              # MSL adds home's own elevation (home_alt, itself MSL) on top, so
              # it's only as accurate as that value (GPS reading or --home-alt-amsl).
              "home_alt_amsl_ft": round(home_alt/FT_TO_M, 2),
              "lap_altitude_agl_ft": round(args.lap_altitude_m/FT_TO_M, 2),
              "lap_altitude_msl_ft": round((home_alt+args.lap_altitude_m)/FT_TO_M, 2),
              "riskmapping_altitude_agl_ft": round(args.altitude_m/FT_TO_M, 2),
              "riskmapping_altitude_msl_ft": round((home_alt+args.altitude_m)/FT_TO_M, 2)}
    out_path.with_suffix(".json").write_text(json.dumps(report, indent=2)+"\n")
    print(f"\nAltitudes (AGL = relative to home, MSL = AGL + home elevation {home_alt/FT_TO_M:.0f} ft):")
    print(f"  Laps:        {args.lap_altitude_m/FT_TO_M:.0f} ft AGL  ({(home_alt+args.lap_altitude_m)/FT_TO_M:.0f} ft MSL)")
    print(f"  Transfer:    {args.lap_altitude_m/FT_TO_M:.0f} ft AGL  ({(home_alt+args.lap_altitude_m)/FT_TO_M:.0f} ft MSL)  (stays at loop altitude)")
    print(f"  Riskmapping: {args.altitude_m/FT_TO_M:.0f} ft AGL  ({(home_alt+args.altitude_m)/FT_TO_M:.0f} ft MSL)")
    print(f"\nWrote {out_path}")
    print("Start capture before arming (Gazebo photos are taken by this script):")
    import shlex
    print(f"python3 mapping_capture.py --polygon {shlex.quote(str(area_path))} "
          f"--capture-interval-m {capture_interval:.3f} --max-tilt-deg 8 "
          f"--min-altitude-m {args.altitude_m*0.95:.2f}")
    if args.format == "mission-planner":
        print("In Mission Planner: PLAN -> right-click map -> Load WP File -> select this file.")
        print("Review the route, then Write WPs to upload to ArduCopter.")
        if fence_path:
            print(f"Fence: {fence_path}")
            print("Select FENCES in PLAN, then Load WP File for the fence and Write.")
            print("Enable polygon fencing and configure breach action in CONFIG/GeoFence.")
    else:
        print("In QGC: Plan view -> File -> Open -> select this file -> Upload (the arrow icon).")


if __name__ == "__main__":
    main()
