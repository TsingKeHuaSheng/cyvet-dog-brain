"""Coordinate transforms and ordered progress on one section of a drawn route."""
import math

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from scipy.spatial.transform import Rotation


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Alignment:
    """A transform with its original observation time, mapping child into parent."""
    def __init__(self, msg):
        self.stamp = stamp_seconds(msg.header.stamp)
        t, q = msg.transform.translation, msg.transform.rotation
        self.translation = np.array([t.x, t.y, t.z])
        self.rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        if not np.isfinite(self.translation).all():
            raise ValueError('Non-finite transform translation')

    def apply(self, xyz):
        return self.rotation.apply(xyz) + self.translation

    def difference_at(self, other, local_point):
        distance = np.linalg.norm(self.apply(local_point) - other.apply(local_point))
        angle = (self.rotation.inv() * other.rotation).magnitude()
        return distance, math.degrees(angle)


class RouteSection:
    def __init__(self, path):
        self.xy = np.array([(p.pose.position.x, p.pose.position.y) for p in path.poses])
        self.arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.xy, axis=0), axis=1))]
        self.orientations = Rotation.from_quat([
            (p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w)
            for p in path.poses])
        self.index = 0

    def update(self, map_position):
        # Follow adjacent samples in order. A crossing later in the stroke cannot
        # become the nearest point just because it has the same map coordinates.
        pos = map_position[:2]
        end = np.searchsorted(self.arc, self.arc[self.index] + 0.25, side='right')
        distances = np.linalg.norm(self.xy[self.index:end] - pos, axis=1)
        best = 0
        while best + 1 < len(distances) and distances[best + 1] < distances[best]:
            best += 1
        distance = float(distances[best])
        if distance <= 0.35:
            self.index = min(self.index + best, len(self.xy) - 2)
        return distance

    def turn_heading(self, local_pose, alignment, lookahead, goal_tolerance):
        # Match the RPP heading target in the local odometry frame, including
        # final orientation when inside the goal position tolerance.
        position = alignment.apply(local_pose.translation)[:2]
        points = self.xy[self.index:]
        if np.linalg.norm(points[-1] - position) < goal_tolerance:
            forward = alignment.rotation.inv().apply(self.orientations[-1].apply([1., 0., 0.]))
        else:
            beyond = np.flatnonzero(np.linalg.norm(points - position, axis=1) >= lookahead)
            target = points[beyond[0]] if len(beyond) else points[-1]
            forward = alignment.rotation.inv().apply([*(target - position), 0.])
        return math.atan2(forward[1], forward[0])

    def local_path(self, alignment):
        # Give Nav2 a path in its local costmap frame. It can now follow fresh
        # lidar -> base_link odometry without waiting for a map TF on each tick.
        path = Path()
        path.header.frame_id = 'lidar'
        xy = self.xy[self.index:]
        xyz = np.column_stack((xy, np.zeros(len(xy))))
        inverse = alignment.rotation.inv()
        positions = inverse.apply(xyz - alignment.translation)
        orientations = (inverse * self.orientations[self.index:]).as_quat()
        for position, orientation in zip(positions, orientations):
            p = PoseStamped()
            p.header.frame_id = path.header.frame_id
            p.pose.position.x, p.pose.position.y, p.pose.position.z = map(float, position)
            q = p.pose.orientation
            q.x, q.y, q.z, q.w = map(float, orientation)
            path.poses.append(p)
        return path


class MotionProgress:
    """Count active tracking time. A turn must reduce a fixed odometric heading error."""
    def __init__(self, radius, angle):
        self.radius = radius
        self.angle = angle
        self.origin = None
        self.target_yaw = None
        self.best_error = None
        self.seconds = 0.

    def sample(self, pose, turning, target_yaw, dt):
        xy = pose.translation[:2]
        self.seconds += dt
        if not turning:
            if self.origin is None:
                self.origin = xy.copy()
            elif np.linalg.norm(xy - self.origin) > self.radius:
                self.origin = xy.copy()
                self.seconds = 0.
            self.target_yaw = None
            return
        self.origin = xy.copy()
        forward = pose.rotation.apply([1., 0., 0.])
        yaw = math.atan2(forward[1], forward[0])
        if (self.target_yaw is None or
                abs(math.remainder(target_yaw - self.target_yaw, 2 * math.pi)) > math.radians(10)):
            # A corrected route may request a different heading. Rebase the
            # error without counting a target change as physical rotation.
            self.target_yaw = target_yaw
            self.best_error = abs(math.remainder(self.target_yaw - yaw, 2 * math.pi))
        error = abs(math.remainder(self.target_yaw - yaw, 2 * math.pi))
        if self.best_error - error >= self.angle:
            self.best_error = error
            self.seconds = 0.
