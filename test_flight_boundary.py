import io
import json
import math
from pathlib import Path
from contextlib import redirect_stdout, redirect_stderr
from unittest.mock import patch
import tempfile
import unittest

from flight_boundary import FlightBoundary
from generate_survey_mission import main


def gps(points):
    return [[47+y/111320, 8+x/(111320*math.cos(math.radians(47)))] for x, y in points]


class BoundaryTests(unittest.TestCase):
    def test_concave_boundary_checks_legs_not_only_waypoints(self):
        boundary = FlightBoundary(gps([(0, 0), (100, 0), (100, 100), (70, 100),
                                       (70, 30), (30, 30), (30, 100), (0, 100)]))
        boundary.check_route(gps([(10, 80), (10, 10), (90, 10), (90, 80)]), 'route')
        with self.assertRaisesRegex(ValueError, 'leg 1'):
            boundary.check_route(gps([(10, 80), (90, 80)]), 'route')
        with self.assertRaisesRegex(ValueError, 'point 1'):
            boundary.check_route(gps([(50, 80)]), 'home')
        # Passing exactly through two concave vertices still leaves the fence.
        with self.assertRaisesRegex(ValueError, 'leg 1'):
            boundary.check_route(gps([(30, 100), (70, 100)]), 'route')

    def test_validation_and_optional_closing_vertex(self):
        square = gps([(0, 0), (100, 0), (100, 100), (0, 100)])
        self.assertEqual(len(FlightBoundary(square+square[:1]).points), 4)
        FlightBoundary(square[::-1]).check_route(gps([(20, 20), (80, 80)]), 'route')
        for points in ([], square[:2], square+[square[1]],
                       gps([(0, 0), (100, 100), (0, 100), (100, 0)]),
                       gps([(0, 0), (50, 0), (100, 0)]), [[math.nan, 8], *square]):
            with self.assertRaises(ValueError):
                FlightBoundary(points)

    def test_cli_fence_and_qgc_export(self):
        config = Path(__file__).parent/'tests/fixtures/mission_example.json'
        vertices = json.loads(config.read_text())['flight_boundary']
        for suffix in ('waypoints', 'plan'):
            with tempfile.TemporaryDirectory() as tmp:
                out = Path(tmp)/f'mission.{suffix}'
                with patch('sys.argv', ['generator', '--mission-config', str(config), '--out', str(out)]), \
                        patch('generate_survey_mission.get_current_position', return_value=(47.3977, 8.5455, 500)), \
                        redirect_stdout(io.StringIO()):
                    main()
                report = json.loads(out.with_suffix('.json').read_text())
                self.assertEqual(report['flight_boundary'], vertices)
                if suffix == 'waypoints':
                    fence = out.with_name('mission_fence.waypoints')
                    self.assertEqual(report['fence_file'], str(fence))
                    rows = [line.split() for line in fence.read_text().splitlines()[1:]]
                    self.assertEqual(len(rows), len(vertices)+1)
                    self.assertEqual(rows[0][:4], ['0', '1', '0', '16'])
                    for index, (row, vertex) in enumerate(zip(rows[1:], vertices), start=1):
                        self.assertEqual(len(row), 12)
                        self.assertEqual(row[:5], [str(index), '0', '0', '5001', str(len(vertices))])
                        self.assertEqual(list(map(float, row[8:10])), vertex)
                    self.assertNotIn('5001', out.read_text())
                else:
                    fence = json.loads(out.read_text())['geoFence']['polygons'][0]
                    self.assertTrue(fence['inclusion'])
                    self.assertEqual(fence['polygon'], vertices)

    def test_outside_home_rejected_before_writing(self):
        config = Path(__file__).parent/'tests/fixtures/mission_example.json'
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/'mission.waypoints'
            with patch('sys.argv', ['generator', '--mission-config', str(config), '--out', str(out)]), \
                    patch('generate_survey_mission.get_current_position', return_value=(47.39, 8.54, 500)), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit) as error:
                main()
            self.assertEqual(error.exception.code, 2)
            self.assertEqual(list(Path(tmp).iterdir()), [])


if __name__ == '__main__':
    unittest.main()
