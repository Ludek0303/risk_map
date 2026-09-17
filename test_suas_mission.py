import io
import json
from contextlib import redirect_stdout
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from generate_suas_mission import main


class SuasMissionTests(unittest.TestCase):
    def test_one_mission_completes_all_laps_before_riskmapping(self):
        original = json.loads((Path(__file__).parent/'tests/fixtures/mission_example.json').read_text())
        for laps in (1, 3):
            with self.subTest(laps=laps), tempfile.TemporaryDirectory() as tmp:
                config = Path(tmp)/'config.json'
                config.write_text(json.dumps(dict(original, laps=laps, lap_altitude_m=60,
                                                   riskmapping_altitude_m=45, lap_speed_m_s=8,
                                                   transfer_speed_m_s=6, riskmapping_speed_m_s=3)))
                out = Path(tmp)/'suas_2026.waypoints'
                with patch('generate_survey_mission.get_current_position',
                           return_value=(47.397742, 8.545594, 500)) as gps, redirect_stdout(io.StringIO()):
                    main(['--mission-config', str(config), '--out', str(out)])
                gps.assert_called_once()
                rows = [list(map(float, line.split())) for line in out.read_text().splitlines()[1:]]
                report = json.loads(out.with_suffix('.json').read_text())
                self.assertEqual(sum(r[3] == 22 for r in rows), 1)
                self.assertEqual(sum(r[3] == 21 for r in rows), 1)
                self.assertEqual(rows[1][3], 22)
                self.assertEqual(rows[-1][3], 21)
                self.assertEqual([r[5] for r in rows if r[3] == 178], [8, 6, 3])
                points = original['lap_points']
                expected = points[:1] + (points[1:]+points[:1])*laps
                # Replay MAV_CMD_DO_JUMP (177) as ArduCopter executes it: the
                # file only writes one pass through the loop, repeated laps
                # are flown by jumping back, not by repeating rows.
                flown_nav_rows = []
                row_index = 1
                jump_counters = {}
                while row_index < len(rows):
                    row = rows[row_index]
                    if row[3] == 177:
                        target, repeat = int(row[4]), int(row[5])
                        remaining = jump_counters.setdefault(row_index, repeat)
                        if remaining > 0:
                            jump_counters[row_index] -= 1
                            row_index = target
                            continue
                        row_index += 1
                        continue
                    if row[3] == 16:
                        flown_nav_rows.append(row)
                    row_index += 1
                for row, point in zip(flown_nav_rows, expected):
                    self.assertEqual(row[10], 60)
                    for actual, desired in zip(row[8:10], point):
                        self.assertAlmostEqual(actual, desired, places=7)
                first_mapping = report['survey_start_item_id']
                self.assertGreater(first_mapping, 2+report['lap_waypoints'])
                mapping_rows = [r for r in rows[first_mapping:-2] if r[3] == 16]
                self.assertEqual(len(mapping_rows), 2*report['lines'])
                self.assertTrue(all(r[10] == 45 for r in mapping_rows))
                self.assertTrue(all(r[3] in (16, 177, 178) for r in rows[2:-1]))
                self.assertEqual([int(r[0]) for r in rows], list(range(len(rows))))
                self.assertEqual(rows[-1][8:10], rows[0][8:10])

    def test_default_profile_and_single_output(self):
        with patch('generate_suas_mission.generate_mission') as generate:
            main([])
        args = generate.call_args.args[0]
        self.assertTrue(args[args.index('--mission-config')+1].endswith('/missions/suas_2026_config.json'))
        self.assertTrue(args[args.index('--out')+1].endswith('/missions/suas_2026.waypoints'))
        self.assertEqual(args[args.index('--format')+1], 'mission-planner')


if __name__ == '__main__':
    unittest.main()
