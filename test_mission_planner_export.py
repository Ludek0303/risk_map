import io
import json
import math
from contextlib import redirect_stdout
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from generate_survey_mission import build_plan, main
from mission_planner_export import mission_planner_text

try:
    from pymavlink import mavwp
except ImportError:
    mavwp = None


class MissionPlannerTests(unittest.TestCase):
    def test_wpl_structure_home_and_relative_altitudes(self):
        plan = build_plan(47.3977, 8.5455, 500, 100, 100, 45, 15, 3,
                          prefix_points=[(0, 0), (10, 0), (0, 0)], lap_altitude_m=60)
        output = mission_planner_text(plan)
        lines = output.splitlines()
        self.assertEqual(lines[0], 'QGC WPL 110')
        rows = [line.split('\t') for line in lines[1:]]
        self.assertTrue(all(len(row) == 12 for row in rows))
        self.assertTrue(all(math.isfinite(float(v)) for row in rows for v in row))
        self.assertEqual([int(r[0]) for r in rows], list(range(len(rows))))
        self.assertEqual(rows[0][:4], ['0', '1', '0', '16'])
        self.assertEqual(list(map(float, rows[0][8:11])), [47.3977, 8.5455, 500])
        self.assertTrue(all(row[1] == '0' and row[2] == '3' for row in rows[1:]))
        self.assertEqual([int(row[3]) for row in rows[1:3]], [22, 178])
        self.assertEqual(list(map(float, rows[2][4:7])), [1, 3, -1])
        self.assertEqual(float(rows[1][10]), 60)
        for row in rows[3:7]:
            # The loop (rows 3-5) plus the transfer arrival (row 6): the
            # transfer flies to the mapping area at loop altitude, then
            # descends in place to survey altitude (row 7) before mapping.
            self.assertEqual(float(row[10]), 60)
        self.assertEqual(rows[6][8:10], rows[7][8:10])
        self.assertEqual(float(rows[7][10]), 45)
        self.assertEqual(rows[-1][3], '21')
        self.assertEqual(list(map(float, rows[-1][8:11])), [47.3977, 8.5455, 0])
        self.assertEqual(list(map(float, rows[-2][8:11])), [47.3977, 8.5455, 45])

    @unittest.skipIf(mavwp is None, 'pymavlink is not installed')
    def test_cli_gps_laps_and_pymavlink_roundtrip(self):
        config = Path(__file__).parent/'tests/fixtures/mission_example.json'
        data = json.loads(config.read_text())
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)/'survey.waypoints'
            stdout = io.StringIO()
            with patch('sys.argv', ['generate_survey_mission.py', '--mission-config', str(config),
                                   '--lap-altitude-m', '60', '--settle-seconds', '2.5',
                                   '--lap-speed', '8', '--cruise-speed', '4', '--transfer-speed', '4',
                                   '--horizontal-accel-m-s2', '2',
                                   '--out', str(output)]), \
                    patch('generate_survey_mission.get_current_position',
                          return_value=(47.3977, 8.5455, 500)) as gps, redirect_stdout(stdout):
                main()
            gps.assert_called_once()
            loader = mavwp.MAVWPLoader()
            loader.load(str(output))
            self.assertEqual(loader.wp(0).z, 500)
            self.assertEqual(loader.wp(1).command, 22)
            # `expected` is the fully unrolled route (every lap), used below
            # for the distance/stop-count estimates; `circuit` is the single
            # pass actually written to the file, repeated via MAV_CMD_DO_JUMP.
            expected = data['lap_points'][:1] + (data['lap_points'][1:]+data['lap_points'][:1])*data['laps']
            circuit = data['lap_points'][:1] + data['lap_points'][1:] + data['lap_points'][:1]
            for i, (lat, lon) in enumerate(circuit, start=3):
                waypoint = loader.wp(i)
                self.assertEqual(waypoint.command, 16)
                self.assertAlmostEqual(waypoint.x, lat, places=7)
                self.assertAlmostEqual(waypoint.y, lon, places=7)
                self.assertEqual(waypoint.z, 60)
                self.assertEqual(waypoint.param1, 3)
            jump_wp = loader.wp(3+len(circuit))
            self.assertEqual(jump_wp.command, 177)
            self.assertEqual(jump_wp.param1, 4)
            self.assertEqual(jump_wp.param2, data['laps']-1)
            report = json.loads(output.with_suffix('.json').read_text())
            self.assertEqual(report['autopilot'], 'arducopter')
            self.assertEqual(report['settle_seconds'], 3)
            self.assertEqual(loader.wp(report['survey_start_item_id']).z, 45)
            self.assertEqual(loader.count(), 7+len(circuit)+1+2*report['lines'])
            speed_items = [loader.wp(i) for i in range(loader.count()) if loader.wp(i).command == 178]
            self.assertEqual([item.param2 for item in speed_items], [8, 4])
            survey_index = report['survey_start_item_id']
            # The transfer leg (survey_index-1) flies at loop altitude and
            # transfer speed; it only descends to survey altitude on arrival
            # (survey_index itself), so the speed change to transfer speed
            # happens one item earlier still (survey_index-2).
            self.assertEqual(loader.wp(survey_index-1).command, 16)
            self.assertEqual(loader.wp(survey_index-1).z, 60)
            self.assertEqual(loader.wp(survey_index-2).command, 178)
            self.assertEqual(report['lap_speed_m_s'], 8)
            self.assertEqual(report['speed_m_s'], 4)
            loop_distance = report['lap_horizontal_distance_m']
            self.assertGreater(loop_distance, 0)
            expected_horizontal = loop_distance/8 + (report['horizontal_distance_m']-loop_distance)/4
            self.assertAlmostEqual(report['estimated_horizontal_seconds'], expected_horizontal)
            stops = len(expected)+2*report['lines']+2
            self.assertAlmostEqual(report['estimated_flight_seconds'], expected_horizontal
                                   + stops*3 + 120/1.5 + report['acceleration_allowance_seconds'])
            # Independently derive the stop-to-stop acceleration estimate from
            # exported waypoints, replaying MAV_CMD_DO_JUMP (177) the way
            # ArduCopter executes it so repeated laps count once per lap.
            from mission_geometry import to_local
            previous = (0., 0.)
            speed = None
            allowance = 0.
            row = 1
            jump_counters = {}
            while row < loader.count():
                wp = loader.wp(row)
                if wp.command == 177:
                    target, repeat = int(wp.param1), int(wp.param2)
                    remaining = jump_counters.setdefault(row, repeat)
                    if remaining > 0:
                        jump_counters[row] -= 1
                        row = target
                        continue
                    row += 1
                    continue
                if wp.command == 178:
                    speed = wp.param2
                elif wp.command == 16:
                    current = to_local((wp.x, wp.y), 47.3977, 8.5455)
                    distance = math.dist(previous, current)
                    leg_time = (2*math.sqrt(distance/2) if distance < speed**2/2
                                else distance/speed + speed/2)
                    allowance += leg_time-distance/speed
                    previous = current
                row += 1
            self.assertAlmostEqual(report['acceleration_allowance_seconds'], allowance, places=5)
            self.assertEqual(loader.wp(loader.count()-1).command, 21)
            self.assertEqual(len(json.loads(output.with_name('survey_area.json').read_text())), 4)
            self.assertIn('Load WP File', stdout.getvalue())

    def test_lap_speed_validation_and_no_laps(self):
        for speed in (0, -1, math.inf, math.nan):
            with self.assertRaises(ValueError):
                build_plan(47, 8, 500, 100, 100, 45, 15, 3, lap_speed=speed)
        plan = build_plan(47, 8, 500, 100, 100, 45, 15, 3, lap_speed=8)
        commands = [item for item in plan['mission']['items'] if item['command'] == 178]
        self.assertEqual(len(commands), 1)
        self.assertEqual(commands[0]['params'][1], 3)

    def test_default_format_is_mission_planner(self):
        with tempfile.TemporaryDirectory() as tmp:
            # Patch only file path resolution so the normal default path is exercised.
            original = Path
            with patch('sys.argv', ['generate_survey_mission.py', '--center-lat', '47', '--center-lon', '8']), \
                    patch('generate_survey_mission.Path', side_effect=lambda p: original(tmp)/p), \
                    redirect_stdout(io.StringIO()):
                main()
            self.assertTrue((Path(tmp)/'missions/survey.waypoints').read_text().startswith('QGC WPL 110\n'))

    def test_wrong_extension_rejected_before_connecting(self):
        for args in (['--out', 'survey.json'], ['--format', 'mission-planner', '--out', 'survey.plan']):
            with patch('sys.argv', ['generate_survey_mission.py', *args]), \
                    patch('generate_survey_mission.get_current_position') as gps, \
                    patch('sys.stderr', new_callable=io.StringIO), self.assertRaises(SystemExit) as error:
                main()
            self.assertEqual(error.exception.code, 2)
            gps.assert_not_called()


if __name__ == '__main__':
    unittest.main()
