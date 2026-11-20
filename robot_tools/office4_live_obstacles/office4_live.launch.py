from pathlib import Path

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    here = Path(__file__).resolve().parent
    return LaunchDescription([
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             namespace='office4_live',
             parameters=[{'min_height': -0.10, 'max_height': 0.35, 'range_min': 0.50,
                          'range_max': 4.25, 'angle_increment': 0.0174533,
                          'concurrency_level': 1}],
             remappings=[('cloud_in', '/fastlio2/body_cloud')]),
        Node(package='nav2_costmap_2d', executable='nav2_costmap_2d',
             parameters=[str(here / 'obstacles.yaml')],
             remappings=[('/costmap/costmap', '/office4_live/costmap'),
                         ('/costmap/costmap_updates', '/office4_live/costmap_updates')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             namespace='office4_live', name='lifecycle', parameters=[
                 {'autostart': True, 'bond_timeout': 0.0, 'node_names': ['/costmap/costmap']}]),
        Node(package='rviz2', executable='rviz2', namespace='office4_live', name='viewer',
             arguments=['-d', str(here / 'Office4_Live.rviz')], on_exit=Shutdown()),
    ])
