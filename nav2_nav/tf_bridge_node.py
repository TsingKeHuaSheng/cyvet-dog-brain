#!/usr/bin/env python3
"""Provide Nav2 /odom with the calibrated base pose and body-frame velocity."""
import argparse
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, str(Path.home() / 'robot_tools/office4_drive'))
from base_odom import BaseOdometry


class FrameBridge(BaseOdometry):
    def __init__(self, args):
        super().__init__(node_name='slam_frame_bridge', base_frame=args.base_frame,
                         output_topic='/odom')
        self.lidar_frame = args.lidar_frame
        self.body_frame = args.body_frame
        self.odom_frame = args.odom_frame
        # FAST-LIO2 already owns lidar -> body, and Localizer owns body -> base_link.
        # The legacy Nav2 odom frame is an alias of lidar and has its own unique parent.
        if self.odom_frame != self.lidar_frame:
            self.alias = StaticTransformBroadcaster(self)
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self.lidar_frame
            transform.child_frame_id = self.odom_frame
            transform.transform.rotation.w = 1.0
            self.alias.sendTransform(transform)

    def sample(self, msg):
        if msg.header.frame_id != self.lidar_frame or msg.child_frame_id != self.body_frame:
            return
        # These two world frames have identical coordinates by construction.
        msg.header.frame_id = self.odom_frame
        super().sample(msg)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--lidar-frame', default='lidar')
    p.add_argument('--body-frame', default='body')
    p.add_argument('--base-frame', default='base_link')
    p.add_argument('--odom-frame', default='odom')
    args = p.parse_args()
    rclpy.init()
    node = FrameBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
