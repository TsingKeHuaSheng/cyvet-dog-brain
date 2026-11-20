"""Office4 stop geometry in the horizontal map plane, expressed in metres."""
import hashlib
from pathlib import Path

import numpy as np
import yaml
from shapely.vectorized import contains
from shapely.geometry import LineString, Polygon


def gate_parameters():
    file = Path(__file__).resolve().parents[1] / 'config/controller.yaml'
    return yaml.safe_load(file.read_text())['/office4_obstacle_gate']['ros__parameters']


def route_xy(path):
    return np.array([(p.pose.position.x, p.pose.position.y) for p in path.poses], dtype='<f8')


def route_id(xy):
    # A decision is valid only for the exact saved stroke the walker loaded.
    return hashlib.sha256(np.asarray(xy, dtype='<f8').tobytes()).hexdigest()


class DetectionZone:
    def __init__(self, xy, params):
        self.route_id = route_id(xy)
        self.corridor = LineString(xy).simplify(1e-9).buffer(
            params['route_half_width'], cap_style=2, join_style=1, resolution=32)
        near = params['nose_from_center']
        far = near + params['front_length']
        half = params['front_half_width']
        self.corners = np.array([[near, -half], [far, -half], [far, half], [near, half]])

    def project(self, center, forward):
        dx, dy = np.asarray(forward[:2]) / np.linalg.norm(forward[:2])
        box = Polygon(self.corners @ [[dx, dy], [-dy, dx]] + center[:2])
        return box, self.corridor.intersection(box)

    @staticmethod
    def hits(overlap, points):
        if overlap.is_empty or not len(points):
            return points[:0]
        # Allow for PointCloud2 float32 rounding at an inclusive boundary.
        return points[contains(overlap.buffer(1e-7), points[:, 0], points[:, 1])]
