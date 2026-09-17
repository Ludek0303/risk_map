import json
from pathlib import Path
import tempfile
import unittest

from mission_geometry import load_config, polygon_grid, read_config_json


class SearchBoundaryTests(unittest.TestCase):
    def test_both_commented_variants(self):
        # Self-contained JSONC template with two area_points blocks (one
        # active, one commented out), independent of missions/suas_2026_config.json
        # so this test doesn't break whenever that file's active boundary changes.
        template = '''{{
  {open1}
  // Search Boundary 1
  "area_points": [[36.0, -96.0], [36.0, -96.01], [36.01, -96.01], [36.01, -96.0]],
  {close1}

  {open2}
  // Search Boundary 2
  "area_points": [[36.02, -96.0], [36.02, -96.01], [36.03, -96.01], [36.03, -96.0]],
  {close2}

  "lap_points": [[36.0, -96.0], [36.0, -96.005]],
  "laps": 1,
  "flight_boundary": [[35.98, -96.02], [35.98, -95.98], [36.05, -95.98], [36.05, -96.02]]
}}'''
        boundary1 = template.format(open1='', close1='', open2='/*', close2='*/')
        boundary2 = template.format(open1='/*', close1='*/', open2='', close2='')
        for source, first in ((boundary1, [36.0, -96.0]), (boundary2, [36.02, -96.0])):
            with tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp)/'config.json'
                path.write_text(source)
                area, _, _, boundary, _ = load_config(path)
                self.assertEqual(list(area[0]), first)
                self.assertEqual(len(boundary.points), 4)
                polygon, _ = polygon_grid(area, *area[0], 20)
                boundary.check_route([*polygon, polygon[0]], 'Search Boundary')

    def test_duplicate_active_areas_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'config.json'
            path.write_text('{"area_points": [], "area_points": []}')
            with self.assertRaisesRegex(ValueError, 'Duplicate'):
                read_config_json(path)

    def test_comment_characters_inside_strings_are_preserved(self):
        data = {'url': 'https://example.com', 'text': 'quote " and /* literal */ // literal'}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'config.json'
            path.write_text('// heading\n'+json.dumps(data)+'\n/* ending */')
            self.assertEqual(read_config_json(path), data)
