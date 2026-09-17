import io
import json
import math
from contextlib import redirect_stdout
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from generate_survey_mission import main
from mission_geometry import load_config


class FlightSettingsTests(unittest.TestCase):
    def run_config(self, changes=None, extra=()):
        data = json.loads((Path(__file__).parent/'tests/fixtures/mission_example.json').read_text())
        data.update(lap_altitude_m=65, riskmapping_altitude_m=40, lap_speed_m_s=8,
                    riskmapping_speed_m_s=3, transfer_speed_m_s=6)
        data.update(changes or {})
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp)/'input.json'
            config.write_text(json.dumps(data))
            out = Path(tmp)/'mission.waypoints'
            with patch('sys.argv', ['generator', '--mission-config', str(config), '--out', str(out), *extra]), \
                    patch('generate_survey_mission.get_current_position', return_value=(47.3977, 8.5455, 500)), \
                    redirect_stdout(io.StringIO()):
                main()
            rows = [list(map(float, line.split())) for line in out.read_text().splitlines()[1:]]
            return rows, json.loads(out.with_suffix('.json').read_text())

    def test_three_speeds_and_two_altitudes_from_json(self):
        rows, report = self.run_config()
        commands = [r for r in rows if r[3] == 178]
        self.assertEqual([r[5] for r in commands], [8, 6, 3])
        self.assertEqual(rows[1][10], 65)
        for row in rows[3:3+report['lap_waypoints']]:
            self.assertEqual(row[10], 65)
        start = report['survey_start_item_id']
        # The transfer leg (start-1) flies at loop altitude and transfer
        # speed; `start` is where it descends in place to survey altitude.
        self.assertEqual(rows[start-2][3:6], [178, 1, 6])
        self.assertEqual(rows[start-1][3], 16)
        self.assertEqual(rows[start-1][10], 65)
        self.assertEqual(rows[start][3], 16)
        self.assertEqual(rows[start][10], 40)
        self.assertEqual(rows[start+1][3:6], [178, 1, 3])
        self.assertEqual(rows[start+2][3], 16)
        self.assertNotEqual(rows[start][8:10], rows[start+2][8:10])
        self.assertEqual(rows[-2][10], 40)
        distance = report['horizontal_distance_m']
        loop = report['lap_horizontal_distance_m']
        transfer = report['transfer_horizontal_distance_m']
        self.assertGreater(transfer, 0)
        self.assertAlmostEqual(report['estimated_horizontal_seconds'],
                               loop/8+transfer/6+(distance-loop-transfer)/3)
        self.assertEqual(report['altitude_m'], 40)
        self.assertEqual(report['lap_altitude_m'], 65)

    def test_cli_overrides_json(self):
        rows, report = self.run_config(extra=('--altitude-m', '50', '--lap-altitude-m', '70',
                                               '--lap-speed', '9', '--cruise-speed', '4',
                                               '--transfer-speed', '7'))
        self.assertEqual([r[5] for r in rows if r[3] == 178], [9, 7, 4])
        self.assertEqual(rows[1][10], 70)
        self.assertEqual(rows[-2][10], 50)
        self.assertEqual(report['transfer_speed_m_s'], 7)

    def test_no_laps_skips_loop_and_transfer_settings(self):
        rows, report = self.run_config({'laps': 0})
        self.assertEqual([r[5] for r in rows if r[3] == 178], [3])
        self.assertEqual(rows[1][10], 40)
        self.assertEqual(report['transfer_horizontal_distance_m'], 0)
        self.assertAlmostEqual(report['estimated_horizontal_seconds'], report['horizontal_distance_m']/3)

    def test_separate_delays_and_time_report(self):
        for laps, mapping in ((0, 2.5), (2.5, 0)):
            with self.subTest(laps=laps, mapping=mapping):
                rows, report = self.run_config({'lap_delay_s': laps, 'riskmapping_delay_s': mapping})
                for row in rows[3:3+report['lap_waypoints']]:
                    self.assertEqual(row[4], math.ceil(laps))
                start = report['survey_start_item_id']
                # The transfer leg (start-1) and the in-place descent to
                # survey altitude (start) both count as arriving at the
                # mapping area, so they use the riskmapping delay, not the
                # loop delay.
                self.assertEqual(rows[start-1][3], 16)
                self.assertEqual(rows[start-1][4], math.ceil(mapping))
                for row in rows[start-1:-1]:
                    if row[3] == 16:
                        self.assertEqual(row[4], math.ceil(mapping))
                self.assertEqual(report['lap_delay_s'], math.ceil(laps))
                self.assertEqual(report['riskmapping_delay_s'], math.ceil(mapping))
                # report['lap_waypoints'] is the physical loop-circuit length
                # written to the file (one pass); repeated laps are flown via
                # MAV_CMD_DO_JUMP, so the *logical* stop count across all laps
                # is (lap_waypoints-1)*laps+1 (only the loop itself uses the
                # loop delay now; the transfer and the descent to survey
                # altitude both count as arriving at the mapping area).
                lap_delay_stops = (report['lap_waypoints']-1)*report['laps'] + 1
                mapping_delay_stops = 2*report['lines'] + 2  # transfer + altitude-change + survey + final home
                expected = (lap_delay_stops*math.ceil(laps)
                            + mapping_delay_stops*math.ceil(mapping))
                self.assertEqual(report['total_delay_seconds'], expected)
                self.assertAlmostEqual(report['estimated_flight_seconds'],
                                       report['estimated_horizontal_seconds']+expected
                                       +report['vertical_distance_m']/1.5)

    def test_common_cli_delay_overrides_both_json_values(self):
        rows, report = self.run_config({'lap_delay_s': 2, 'riskmapping_delay_s': 5},
                                       extra=('--settle-seconds', '0'))
        self.assertTrue(all(r[4] == 0 for r in rows if r[3] == 16))
        self.assertEqual(report['total_delay_seconds'], 0)

    def test_invalid_json_delays(self):
        base = json.loads((Path(__file__).parent/'tests/fixtures/mission_example.json').read_text())
        for name in ('lap_delay_s', 'riskmapping_delay_s'):
            for value in (-1, math.nan, math.inf, True, '3', None):
                with self.subTest(name=name, value=value), tempfile.TemporaryDirectory() as tmp:
                    config = Path(tmp)/'input.json'
                    config.write_text(json.dumps(dict(base, **{name: value})))
                    with self.assertRaisesRegex(ValueError, name):
                        load_config(config)

    def test_invalid_json_settings(self):
        base = json.loads((Path(__file__).parent/'tests/fixtures/mission_example.json').read_text())
        for name in ('lap_altitude_m', 'riskmapping_altitude_m', 'lap_speed_m_s',
                     'riskmapping_speed_m_s', 'transfer_speed_m_s'):
            for value in (0, -1, math.nan, math.inf, True, '6', None):
                with self.subTest(name=name, value=value), tempfile.TemporaryDirectory() as tmp:
                    config = Path(tmp)/'input.json'
                    config.write_text(json.dumps(dict(base, **{name: value})))
                    with self.assertRaisesRegex(ValueError, name):
                        load_config(config)


if __name__ == '__main__':
    unittest.main()
