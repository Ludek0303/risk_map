import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from generate_survey_mission import build_grid_lines, build_plan


class SurveyMissionTests(unittest.TestCase):
    def test_grid_inside_polygon_and_no_gaps(self):
        lines = build_grid_lines(147, 262, 15.8)
        self.assertEqual(len(lines), 11)
        for line in lines:
            for east, north in line:
                self.assertLess(abs(east), 147/2)
                self.assertLess(abs(north), 262/2)
        for a, b in zip(lines, lines[1:]):
            self.assertLessEqual(b[0][0]-a[0][0], 15.8)
            self.assertEqual(a[-1][1], b[0][1])

    def test_speed_climb_and_return(self):
        mission = build_plan(37, -122, 38, 147, 262, 45, 15.8, 3)["mission"]
        items = mission["items"]
        self.assertEqual(items[0]["params"][4:], [37, -122, 45])
        self.assertEqual(items[1]["command"], 178)
        self.assertEqual(items[1]["params"][:3], [1, 3, -1])
        self.assertEqual(mission["hoverSpeed"], 3)
        self.assertEqual(items[-2]["params"][4:], [37, -122, 45])
        self.assertEqual(items[-1]["command"], 21)
        self.assertEqual(items[-1]["params"][4:], [37, -122, 0])
        self.assertEqual([i["doJumpId"] for i in items], list(range(1, len(items)+1)))
        for item in items[2:-1]:
            self.assertEqual(item["params"][0], 3)
            self.assertEqual(item["params"][6], 45)

    def test_fractional_speed_metadata_is_importable_without_changing_flight_speed(self):
        for speed in (7.4, 3.0, 0.4):
            with self.subTest(speed=speed):
                plan = build_plan(37, -122, 38, 147, 262, 45, 15.8, speed,
                                  prefix_points=[(0, 0), (10, 10), (0, 0)],
                                  lap_speed=8.2, transfer_speed=6.3)
                mission = json.loads(json.dumps(plan))['mission']
                for field in ('cruiseSpeed', 'hoverSpeed'):
                    self.assertIs(type(mission[field]), int)
                    self.assertGreater(mission[field], 0)
                commands = [i for i in mission['items'] if i['command'] == 178]
                self.assertEqual([i['params'][1] for i in commands], [8.2, 6.3, speed])

    def test_bad_geometry_is_rejected(self):
        for spacing in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                build_grid_lines(147, 262, spacing)

    def test_offline_cli_frontlap_and_companion_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)/"survey.plan"
            result = subprocess.run([sys.executable, str(Path(__file__).with_name("generate_survey_mission.py")),
                "--center-lat", "37", "--center-lon", "-122", "--frontlap", "0.98",
                "--out", str(output)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads(output.with_suffix(".json").read_text())
            self.assertGreaterEqual(report["frontlap_min"], 0.98)
            self.assertGreaterEqual(report["sidelap_min"], 0.8)
            self.assertLess(report["capture_interval_m"], 5)
            self.assertEqual(len(json.loads(output.with_name("survey_area.json").read_text())), 4)
            self.assertIn("--min-altitude-m 42.75", result.stdout)


if __name__ == "__main__":
    unittest.main()
