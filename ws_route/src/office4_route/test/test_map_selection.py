import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import map_site
import route_map


class MapSelectionTest(unittest.TestCase):
    def test_map_names(self):
        self.assertEqual(map_site.map_name('Office_5-demo'), 'Office_5-demo')
        for name in ('', '../office4', 'run/1', '/run1'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                map_site.map_name(name)

    def test_saved_route_requires_same_map_and_cloud(self):
        with tempfile.TemporaryDirectory() as root:
            file = Path(root) / 'routes/hand_drawn.json'
            with patch.object(route_map, 'MAP_NAME', 'RUN1'):
                route_map.save_route(file, [[0, 0], [1, 0]], 'map-one')
                self.assertEqual(route_map.load_route(file, 'map-one').poses[-1].pose.position.x, 1)
                with self.assertRaises(ValueError):
                    route_map.load_route(file, 'map-two')
            with patch.object(route_map, 'MAP_NAME', 'RUN2'), self.assertRaises(ValueError):
                route_map.load_route(file, 'map-one')

    def test_floor_reference_is_site_specific(self):
        with tempfile.TemporaryDirectory() as root:
            site = Path(root)
            for folder, name, height in [('nav2', 'other_site', -1.2),
                                         ('navigation_preview', str(site.resolve()), -0.46)]:
                dest = site / folder
                dest.mkdir()
                (dest / 'projection.json').write_text(json.dumps({'source': name, 'floor_z': height}))
            self.assertEqual(map_site.floor_height(site), -0.46)
