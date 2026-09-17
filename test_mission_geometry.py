import json
import math
from pathlib import Path
import io
from contextlib import redirect_stdout
from unittest.mock import patch
import sys
import tempfile
import unittest

from area import GpsPolygon
from generate_survey_mission import meters_to_latlon_offset, main, build_plan
from mission_geometry import load_config, polygon_grid
from tests.mission_sim import flown_items


class FourCornerTests(unittest.TestCase):
    def test_clipped_trapezoid_and_unordered_corners(self):
        points = [[47.3981, 8.5452], [47.3998, 8.5469],
                  [47.3982, 8.5471], [47.3996, 8.5454]]
        polygon, lines = polygon_grid(points, 47.397742, 8.545594, 15)
        area = GpsPolygon(polygon)
        self.assertGreater(len(lines), 2)
        for i, line in enumerate(lines):
            self.assertEqual(line[1][1] > line[0][1], i % 2 == 0)
            for east, north in line:
                dlat, dlon = meters_to_latlon_offset(47.397742, east, north)
                self.assertTrue(area.contains(47.397742+dlat, 8.545594+dlon))
        for a, b in zip(lines, lines[1:]):
            self.assertLessEqual(b[0][0]-a[0][0], 15+1e-9)

    def test_bad_corners(self):
        for points in ([[47, 8]]*4,
                       [[47, 8], [47, 8], [48, 8], [48, 9]],
                       [[47, 8], [47, 8.01], [47.01, 8], [47.001, 8.001]],
                       [[47, 8], [47, 8.01], [47, 8.02], [47, 8.03]]):
            with self.assertRaises((ValueError, TypeError)):
                polygon_grid(points, 47, 8, 15)

    def test_complete_laps_before_survey_and_report(self):
        root = Path(__file__).parent
        config_path = root/'tests/fixtures/mission_example.json'
        config = json.loads(config_path.read_text())
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)/'flight.plan'
            home = [47.3977, 8.5455]
            with patch('sys.argv', ['generate_survey_mission.py', '--mission-config',
                                   str(config_path), '--lap-altitude-m', '60', '--out', str(output)]), \
                    patch('generate_survey_mission.get_current_position',
                          return_value=(*home, 500)) as position, redirect_stdout(io.StringIO()):
                main()
            position.assert_called_once()
            plan = json.loads(output.read_text())['mission']
            items = plan['items']
            lap = config['lap_points']
            laps = config['laps']
            circuit = lap[:1] + lap[1:] + lap[:1]  # one full circuit: only laps=1 is written out
            loop_items = items[2:2+len(circuit)]
            for item, point in zip(loop_items, circuit):
                self.assertEqual(item['command'], 16)
                for actual, wanted in zip(item['params'][4:6], point):
                    self.assertAlmostEqual(actual, wanted)
            # Repeated laps are flown by jumping back to the circuit's second
            # point (row = 1-based position in `items`, since row 0 is home)
            # rather than writing every lap out again.
            jump_item = items[2+len(circuit)]
            self.assertEqual(jump_item['command'], 177)
            self.assertEqual(jump_item['params'][0], 4)
            self.assertEqual(jump_item['params'][1], laps - 1)
            self.assertEqual(plan['plannedHomePosition'], [*home, 500])
            self.assertEqual(items[0]['params'][6], 60)
            for item in loop_items:
                self.assertEqual(item['params'][6], 60)
                self.assertEqual(item['Altitude'], 60)
            # The transfer leg flies at loop altitude and only descends to
            # survey altitude once it arrives at the mapping area.
            transfer = items[3+len(circuit)]
            self.assertEqual(transfer['params'][6], 60)
            transition = items[4+len(circuit)]
            self.assertEqual(transition['params'][4:6], transfer['params'][4:6])
            self.assertEqual(transition['params'][6], 45)
            for item in items[5+len(circuit):-1]:
                self.assertEqual(item['params'][6], 45)
            report = json.loads(output.with_suffix('.json').read_text())
            self.assertEqual(report['vertical_distance_m'], 120)
            self.assertEqual(report['lap_waypoints'], len(circuit))
            self.assertEqual(report['survey_start_item_id'], 5 + len(circuit))
            self.assertEqual(items[-1]['params'][4:6], home)
            self.assertEqual([i['doJumpId'] for i in items], list(range(1, len(items)+1)))
            self.assertEqual(len(items), 5 + len(circuit) + 1 + 2*report['lines'])
            # Replay MAV_CMD_DO_JUMP as ArduCopter would, to check the
            # reported flight distance against the path actually flown
            # (all `laps` repeats), not just the physical file rows.
            flown = flown_items(items)
            coords = [i['params'][4:6] for i in flown]
            from mission_geometry import to_local
            local = [to_local(p, *home) for p in coords]
            self.assertAlmostEqual(report['horizontal_distance_m'],
                                   sum(math.dist(a, b) for a, b in zip(local, local[1:])), places=5)

    def test_invalid_lap_altitude(self):
        for height in (0, -10, math.nan, math.inf):
            with self.assertRaises(ValueError):
                build_plan(47, 8, 500, 100, 100, 45, 15, 3, lap_altitude_m=height)

    def test_invalid_config(self):
        base = json.loads((Path(__file__).parent/'tests/fixtures/mission_example.json').read_text())
        for changes in ({'laps': -1}, {'laps': 1.5}, {'laps': True},
                        {'lap_points': []}, {'home': [90, 0]},
                        {'area_points': [[47, 8]]}, {'lap_points': [[math.nan, 8]]}):
            with tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp)/'input.json'
                path.write_text(json.dumps(dict(base, **changes)))
                with self.assertRaises(ValueError):
                    load_config(path)


if __name__ == '__main__':
    unittest.main()
