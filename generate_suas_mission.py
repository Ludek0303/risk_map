"""Generate one continuous SUAS route: laps, transfer, riskmapping, return.

Edit missions/suas_2026_config.json, then run this script with the drone
connected before takeoff. Home is read from live GPS by the shared generator.
All arguments accepted by generate_survey_mission.py may be supplied here.
"""
from pathlib import Path
import sys

from generate_survey_mission import main as generate_mission


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    root = Path(__file__).resolve().parent
    defaults = []
    for option, value in (
        ('--mission-config', root/'missions/suas_2026_config.json'),
        ('--out', root/'missions/suas_2026.waypoints'),
        ('--format', 'mission-planner'),
    ):
        if not any(arg == option or arg.startswith(option+'=') for arg in args):
            defaults.extend([option, str(value)])
    generate_mission([*defaults, *args])


if __name__ == '__main__':
    main()
