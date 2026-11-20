#!/usr/bin/env python3
"""RViz /goal_pose -> path preview only. No control client or velocity publisher."""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose


class GoalToPlan(Node):
    def __init__(self):
        super().__init__('goal_to_plan_preview')
        self.client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Path, '/navigation_preview/path', qos)
        self.subscription = self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.busy = False
        self.get_logger().info('Planning preview: RViz /goal_pose -> /navigation_preview/path. No motion output.')

    def clear(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(path)

    def on_goal(self, pose):
        if self.busy:
            self.get_logger().warning('Still planning; wait before selecting another goal.')
            return
        self.clear()
        if pose.header.frame_id != 'map':
            self.get_logger().error('Goal must use map frame; set RViz Fixed Frame to map.')
            return
        if not self.client.server_is_ready():
            self.get_logger().warning('Planner server not ready.')
            return
        goal = ComputePathToPose.Goal()
        goal.goal = pose
        goal.planner_id = 'GridBased'
        goal.use_start = False  # Planner obtains live map -> base_link from TF.
        self.busy = True
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.accepted)

    def accepted(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.busy = False
                self.get_logger().error('Planning request rejected.')
                return
            handle.get_result_async().add_done_callback(self.finished)
        except Exception as exc:
            self.busy = False
            self.get_logger().error(str(exc))

    def finished(self, future):
        self.busy = False
        try:
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.path.poses:
                self.publisher.publish(result.result.path)
                self.get_logger().info(f'Preview path: {len(result.result.path.poses)} poses. Robot remains uncontrolled.')
            else:
                self.clear()
                self.get_logger().error(f'No valid path; action status={result.status}. Check map, goal and clearance.')
        except Exception as exc:
            self.get_logger().error(str(exc))


def main():
    rclpy.init()
    node = GoalToPlan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
