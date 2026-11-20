"""Regression checks for slow turns, false progress, and ordered rejoining."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from route_map import route_message
from route_tracking import MotionProgress, RouteSection


def pose(yaw, x=0.0):
    return SimpleNamespace(rotation=Rotation.from_euler('z', yaw),
                           translation=np.array([x, 0., 0.]))


class RouteProgressTests(unittest.TestCase):
    def test_turn_toward_heading_crosses_pi_and_outlasts_translation_timeout(self):
        progress = MotionProgress(.06, .05)
        for step in range(80):
            progress.sample(pose(2.5 + step * .01), True, -2.75, .5)
            self.assertLess(progress.seconds, 4)

    def test_wrong_way_turn_and_chassis_wobble_do_not_count(self):
        progress = MotionProgress(.06, .05)
        for step in range(60):
            progress.sample(pose(-.01 * step, .08 * (-1)**step), True, 1., .5)
        self.assertGreater(progress.seconds, 25)

    def test_map_changes_without_body_rotation_do_not_count(self):
        progress = MotionProgress(.06, .05)
        for step in range(60):
            progress.sample(pose(0.), True, math.radians(50 if step % 2 else 80), .5)
        self.assertGreater(progress.seconds, 25)

    def test_crossing_never_rejoins_distant_pass(self):
        section = RouteSection(route_message([[-1, 0], [1, 0], [0, 1], [0, -1]]))
        for x in np.linspace(-1, 0, 40):
            section.update(np.array([x, 0.]))
        idx = section.index
        for _ in range(20):
            section.update(np.array([0., .23]))
        self.assertEqual(idx, section.index)
        self.assertGreater(section.update(np.array([0., .6])), .35)
        self.assertEqual(idx, section.index)


if __name__ == '__main__':
    unittest.main()
