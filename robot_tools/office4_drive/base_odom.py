from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation


class BaseOdometry(Node):
    def __init__(self, *, node_name='office4_base_odom', base_frame='base_link',
                 output_topic='/office4_drive/base_odom'):
        super().__init__(node_name)
        self.base_frame = base_frame
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.history = deque()
        self.pub = self.create_publisher(Odometry, output_topic, 5)
        self.sub = self.create_subscription(Odometry, '/fastlio2/lio_odom', self.sample, qos_profile_sensor_data)

    def sample(self, msg):
        try:
            offset = self.tf.lookup_transform(msg.child_frame_id, self.base_frame, Time()).transform
        except TransformException:
            return
        pose = msg.pose.pose
        q = pose.orientation
        body = Rotation.from_quat([q.x,q.y,q.z,q.w])
        t = offset.translation
        xyz = np.array([pose.position.x,pose.position.y,pose.position.z]) + body.apply([t.x,t.y,t.z])
        q = offset.rotation
        base = body * Rotation.from_quat([q.x,q.y,q.z,q.w])
        ts = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.history and (ts <= self.history[-1][0] or ts-self.history[-1][0] > 0.8):
            self.history.clear()
        self.history.append((ts,xyz,base))
        while len(self.history)>1 and ts-self.history[0][0]>0.55:
            self.history.popleft()
        start,p0,r0 = self.history[0]
        dt = ts-start
        if dt < 0.20:
            return
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z = xyz
        q = out.pose.pose.orientation
        q.x,q.y,q.z,q.w = base.as_quat()
        linear = out.twist.twist.linear
        linear.x,linear.y,linear.z = base.inv().apply((xyz-p0)/dt)
        angular = out.twist.twist.angular
        angular.x,angular.y,angular.z = (r0.inv()*base).as_rotvec()/dt
        self.pub.publish(out)

if __name__ == '__main__':
    rclpy.init()
    rclpy.spin(BaseOdometry())
