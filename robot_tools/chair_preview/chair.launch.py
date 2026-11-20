from pathlib import Path

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    folder = Path(__file__).resolve().parent
    ns = 'chair_preview'
    tf = [('/tf', '/chair_preview/tf'), ('/tf_static', '/chair_preview/tf_static')]
    return LaunchDescription([
        Node(package='livox_pc2_bridge', executable='livox_pc2_bridge_node', namespace=ns,
             remappings=[('/livox/lidar_pc2', '/chair_preview/cloud')]),
        Node(package='tf2_ros', executable='static_transform_publisher', namespace=ns,
             arguments=['--frame-id', 'chair_view', '--child-frame-id', 'livox_frame'], remappings=tf),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node', namespace=ns,
             parameters=[{'min_height': -0.10, 'max_height': 0.25, 'range_min': 0.45, 'range_max': 4.5,
                          'angle_increment': 0.0174533, 'concurrency_level': 1}],
             remappings=[('cloud_in', '/chair_preview/cloud'), ('scan', '/chair_preview/scan')] + tf),
        Node(package='nav2_costmap_2d', executable='nav2_costmap_2d', name='costmap', namespace=ns,
             parameters=[str(folder / 'costmap.yaml')],
             remappings=tf + [('/costmap/costmap', '/chair_preview/costmap'),
                              ('/costmap/costmap_updates', '/chair_preview/costmap_updates')], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', namespace=ns,
             name='lifecycle', parameters=[{'autostart': True, 'bond_timeout': 0.0,
                                            'node_names': ['/costmap/costmap']}]),
        Node(package='rviz2', executable='rviz2', name='viewer', namespace=ns,
             arguments=['-d', str(folder / 'Office4_Live_Obstacles.rviz')], remappings=tf,
             on_exit=Shutdown()),
    ])
