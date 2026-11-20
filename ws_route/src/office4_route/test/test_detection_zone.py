"""Run with the Office4 ROS environment sourced."""
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from detection_zone import DetectionZone, gate_parameters
from obstacles import base_in_body_xy, classify
from zone_display import active_markers
from builtin_interfaces.msg import Time


class ZoneTest(unittest.TestCase):
    def setUp(self):
        self.params = dict(nose_from_center=.315, front_length=.40, front_half_width=.30,
                           route_half_width=.30, stop_points=3)

    def test_installed_front_length(self):
        params = gate_parameters()
        zone = DetectionZone([[0., 0.], [2., 0.]], params)
        box, _ = zone.project([0., 0.], [1., 0.])
        self.assertAlmostEqual(box.bounds[2] - box.bounds[0], params['front_length'])

    def test_nose_edges_width_and_count(self):
        zone = DetectionZone([[-1, 0], [3, 0]], self.params)
        box, overlap = zone.project(np.zeros(3), [1., 0.])
        np.testing.assert_allclose(box.bounds, [.315, -.30, .715, .30])
        pts = np.array([[.315, -.30, .35], [.715, .30, .65], [.316, 0, .5],
                        [.314, 0, .5], [.716, 0, .5], [.50, .301, .5],
                        [.50, -.301, .5]], dtype=np.float32)
        np.testing.assert_array_equal(zone.hits(overlap, pts), pts[:3])
        self.assertEqual(self.params['stop_points'], 3)
        self.assertEqual(len(zone.hits(overlap, pts[:2])), 2)
        self.assertEqual(len(zone.hits(overlap, pts[:0])), 0)

    def test_bend_and_route_end(self):
        zone = DetectionZone([[0, 0], [.35, 0], [.35, 1]], self.params)
        box, overlap = zone.project([0., 0.], [1., 0.])
        pts = np.array([[.5, .1, .3], [.69, 0, .4], [.36, .34, .4], [.60, -.28, .4]])
        np.testing.assert_array_equal(zone.hits(overlap, pts), pts[:1])
        translated, clipped = zone.project([.35, .1], [0., 1.])
        np.testing.assert_allclose(translated.bounds, [.05, .415, .65, .815])
        self.assertAlmostEqual(clipped.area, .24)
        end_box, empty = zone.project([.35, 1.], [0., 1.])
        self.assertTrue(empty.is_empty)
        markers = active_markers(end_box, empty, Time())
        self.assertEqual(markers.markers[1].action, markers.markers[1].DELETE)

    def test_double_pass_counts_each_return_once(self):
        zone = DetectionZone([[0, 0], [2, 0], [2, .18], [0, .18]], self.params)
        _, overlap = zone.project([0, 0], [1., 0.])
        pts = np.array([[.4, .09, .3], [.5, .09, .3]])
        self.assertEqual(len(zone.hits(overlap, pts)), 2)

    def test_height_filtered_returns_immediately_beyond_nose(self):
        zone = DetectionZone([[0, 0], [2, 0]], self.params)
        _, overlap = zone.project([0., 0.], [1., 0.])
        nose = self.params['nose_from_center']
        for dx, height, count in [(.01, .55, 3), (.05, .55, 3), (-.01, .55, 0),
                                  (.01, .299, 0), (.01, .301, 3),
                                  (.01, .899, 3), (.01, .901, 0)]:
            with self.subTest(dx=dx, height=height):
                local = np.array([[nose + dx, y, height - .43] for y in (-.005, 0., .005)])
                local[:, :2] += base_in_body_xy
                marked, _ = classify(local, Rotation.identity(), [0., 0., -.09], -.52)
                marked[:, :2] -= base_in_body_xy
                self.assertEqual(len(zone.hits(overlap, marked)), count)


if __name__ == '__main__':
    unittest.main()
