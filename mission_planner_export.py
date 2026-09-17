"""Export this project's multirotor route as a Mission Planner WPL 110 file.

Format: https://mavlink.io/en/file_formats/
Row zero is planned home in AMSL; flight rows use relative-to-home altitude.
This exporter targets ArduCopter, not Plane or Rover.
"""
import math


def mission_planner_text(plan):
    mission = plan['mission']
    lat, lon, altitude = mission['plannedHomePosition']
    rows = [[0, 1, 0, 16, 0, 0, 0, 0, lat, lon, altitude, 1]]
    for index, item in enumerate(mission['items'], start=1):
        command = item['command']
        if command not in (16, 21, 22, 177, 178):
            raise ValueError(f'Unsupported ArduCopter export command: {command}')
        params = list(item['params'])
        if len(params) != 7:
            raise ValueError('Mission items must have seven parameters')
        # Empty fields in Mission Planner are numeric zeros. ArduCopter's
        # waypoint acceptance radius is controlled by its own parameters.
        params = [0 if value is None else value for value in params]
        if command == 16:
            params[1:4] = [0, 0, 0]
            if not 0 <= params[0] <= 65535 or params[0] != int(params[0]):
                raise ValueError('ArduCopter waypoint delay must be an integer in [0, 65535]')
        # Mission Planner stores DO items with the relative-alt frame too;
        # their zero location fields are unused by DO_CHANGE_SPEED.
        rows.append([index, 0, 3, command, *params, int(item['autoContinue'])])
    for row in rows:
        if not all(math.isfinite(value) for value in row):
            raise ValueError('Mission Planner fields must be finite numbers')
    return 'QGC WPL 110\n' + ''.join(
        '\t'.join(format(value, '.12g') for value in row) + '\n' for row in rows)


def mission_planner_fence_text(plan, boundary):
    """Mission Planner WPL file: home placeholder followed by inclusion vertices.

    Load with FENCES selected; MP strips row zero before fence upload.
    """
    lat, lon, altitude = plan['mission']['plannedHomePosition']
    rows = [[0, 1, 0, 16, 0, 0, 0, 0, lat, lon, altitude, 1]]
    for index, (lat, lon) in enumerate(boundary.points, start=1):
        rows.append([index, 0, 0, 5001, len(boundary.points), 0, 0, 0, lat, lon, 0, 1])
    return 'QGC WPL 110\n' + ''.join(
        '\t'.join(format(value, '.12g') for value in row) + '\n' for row in rows)
