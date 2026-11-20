"""Classify Office4 clouds by height above the mapped floor, in a gravity-aligned frame."""
import json
from collections import deque
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Path as Route
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from detection_zone import DetectionZone, gate_parameters, route_xy
from zone_display import active_markers, outline
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformException
from route_map import xyz, cloud_message
from route_tracking import Alignment


geometry_path = Path(get_package_share_directory('localizer')) / 'config/base_link_reference.json'
geometry = json.loads(geometry_path.read_text())
mount = geometry['lidar_mount_extrinsic']
base_in_body_xy = (np.array(mount['mount_center_to_base_link_m']) -
                   np.array(mount['mount_center_to_body_imu_m']))[:2]
robot = geometry['assembled_robot_m']
# Exclude physical body returns. The space immediately ahead of the nose is observable.
self_half_size = np.array([robot['length'], robot['width']]) / 2


def classify(points, rotation, origin, floor_z):
    # Only estimate a local floor correction from observations close to the known
    # Office4 floor; the supported fallback in this flat office is its mapped floor.
    delta = rotation.apply(points)
    heights = delta[:,2]+origin[2]
    radial = np.linalg.norm(delta[:,:2],axis=1)
    ground = (radial<2.5)&(radial>0.55)&(np.abs(heights-floor_z)<0.08)
    floor = floor_z
    if ground.sum()>=20:
        cells = np.floor(delta[ground,:2]/0.25).astype(int)
        if len(np.unique(cells,axis=0))>=8:
            floor=float(np.median(heights[ground]))
    h=heights-floor
    # Both frames are axis-aligned in the installed calibration.
    outside = np.any(np.abs(points[:, :2] - base_in_body_xy) >= self_half_size - 1e-7, axis=1)
    # A real return just beyond the nose can be within 12 cm horizontally
    # of the lidar. Keep it for marking once it is outside the measured body.
    usable = outside & (radial <= 4.5)
    mark = usable&(h>=0.30)&(h<=0.90)
    clear = usable&(radial>=0.12)&(h>=0.0)&(h<=0.90)
    return points[mark],points[clear]


class Obstacles(Node):
    def __init__(self):
        super().__init__('office4_height_filter')
        self.floor=self.declare_parameter('floor_z', -0.52).value
        self.alignment = None
        self.tf=Buffer()
        self.listen=TransformListener(self.tf,self)
        self.pending=deque(maxlen=20)
        self.mark=self.create_publisher(PointCloud2,'/line_route/obstacles',qos_profile_sensor_data)
        self.clear=self.create_publisher(PointCloud2,'/line_route/clearing_rays',qos_profile_sensor_data)
        self.params = gate_parameters()
        self.zone = None
        self.base_in_body = None
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.route_outline = self.create_publisher(Marker, '/line_route/route_corridor', durable)
        self.active = self.create_publisher(MarkerArray, '/line_route/detection_zone', 1)
        self.hits = self.create_publisher(PointCloud2, '/line_route/front_hits', qos_profile_sensor_data)
        self.decision = self.create_publisher(DiagnosticArray, '/line_route/obstacle_gate', 1)
        self.create_subscription(Route, '/line_route/saved', self.set_route, durable)
        self.create_subscription(PointCloud2,'/fastlio2/body_cloud',self.pending.append,qos_profile_sensor_data)
        self.create_subscription(TransformStamped, '/localizer/accepted_alignment', self.align, 1)
        self.create_timer(.04,self.update)

    def set_route(self, msg):
        self.zone = None
        if msg.header.frame_id == 'map' and len(msg.poses) >= 2:
            self.zone = DetectionZone(route_xy(msg), self.params)
            self.route_outline.publish(outline(
                self.zone.corridor, 'route_corridor', (0., .85, .9), .012))

    def evaluate(self, msg, marks, rotation, origin):
        if self.zone is None:
            return
        if self.base_in_body is None:
            try:
                self.base_in_body = Alignment(self.tf.lookup_transform('body', 'base_link', Time()))
            except TransformException:
                return
        center = rotation.apply(self.base_in_body.translation) + origin
        forward = (rotation * self.base_in_body.rotation).apply([1., 0., 0.])
        box, overlap = self.zone.project(center, forward)
        mapped = rotation.apply(marks) + origin if len(marks) else marks
        hits = self.zone.hits(overlap, mapped)
        blocked = len(hits) >= self.params['stop_points']
        header = Header(stamp=msg.header.stamp, frame_id='map')
        status = DiagnosticStatus(name='office4_route/obstacle_gate',
                                  level=DiagnosticStatus.WARN if blocked else DiagnosticStatus.OK,
                                  message='occupied' if blocked else 'clear',
                                  values=[KeyValue(key='route_id', value=self.zone.route_id),
                                          KeyValue(key='hit_count', value=str(len(hits)))])
        self.decision.publish(DiagnosticArray(header=header, status=[status]))
        self.hits.publish(cloud_message(header, hits if blocked else hits[:0]))
        self.active.publish(active_markers(box, overlap, msg.header.stamp))

    def align(self, msg):
        if (msg.header.frame_id, msg.child_frame_id) == ('map', 'lidar'):
            self.alignment = Alignment(msg)
        else:
            self.alignment = None
            self.pending.clear()

    def update(self):
        while self.pending and self.alignment is not None:
            m=self.pending[0]
            stamp=Time.from_msg(m.header.stamp)
            age=(self.get_clock().now()-stamp).nanoseconds*1e-9
            if m.header.frame_id!='body' or age>0.85 or age < -0.1:
                self.pending.popleft()
                continue
            try:
                local = Alignment(self.tf.lookup_transform('lidar', 'body', stamp))
            except TransformException:
                return
            self.pending.popleft()
            # The site floor is recorded in map coordinates. Use the cached
            # alignment and the attitude at scan time, also between ICP updates.
            rot = self.alignment.rotation * local.rotation
            origin = self.alignment.apply(local.translation)
            points = xyz(m)
            if not len(points):
                continue
            mark,clear=classify(points,rot,origin,self.floor)
            self.evaluate(m, mark, rot, origin)
            self.clear.publish(cloud_message(m.header,clear))
            self.mark.publish(cloud_message(m.header,mark))


if __name__=='__main__':
    rclpy.init()
    rclpy.spin(Obstacles())
