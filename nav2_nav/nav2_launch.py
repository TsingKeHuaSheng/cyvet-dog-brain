#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cyvet Nav2 导航层总启动 (Jetson 外挂板)

前置 (不由本 launch 启动):
  1. livox-lidar.service          (systemd 自启)
  2. localizer_launch.py          (lio_node + localizer + rviz2)
  3. uniubi_motion_bridge         (cmd_vel -> 狗, 注意独立域问题)

本 launch 启动:
  - tf_bridge_node.py             发布 lidar 到 odom 的同原点别名，按已标定的机身参考点发布 /odom
  - pointcloud_to_laserscan       /fastlio2/body_cloud -> /scan
  - map_server + lifecycle_manager
  - nav2_bringup navigation_launch.py (controller/planner/bt/behavior/waypoint)

用法:
  ros2 launch cyvet_nav2 nav2_launch.py map:=/home/nvidia/robot_ws/maps/run3/grid/map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

THIS_DIR = os.path.dirname(os.path.realpath(__file__))
DEFAULT_PARAMS = os.path.join(THIS_DIR, 'nav2_params.yaml')
DEFAULT_MAP = '/home/nvidia/robot_ws/maps/run3/grid/map.yaml'


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    pkg_nav2 = get_package_share_directory('nav2_bringup')

    declare = [
        DeclareLaunchArgument(
            'params_file', default_value=DEFAULT_PARAMS),
        DeclareLaunchArgument('map', default_value=DEFAULT_MAP),
    ]

    # 1. SLAM->Nav2 帧桥
    tf_bridge = ExecuteProcess(
        cmd=['python3', os.path.join(THIS_DIR, 'tf_bridge_node.py')],
        output='screen')

    # 2. 点云切片 -> 模拟激光
    scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[('cloud_in', '/fastlio2/body_cloud')],
        parameters=[params_file],
        output='screen')

    # 3. 静态地图
    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server',
        parameters=[params_file, {'yaml_filename': map_yaml}],
        output='screen')

    map_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 0.0,
        }],
        output='screen')

    # 4. Nav2 导航栈 (controller/planner/bt_navigator/behavior/waypoint)
    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': params_file,
            'use_composition': 'False',
        }.items())

    return LaunchDescription(declare + [
        tf_bridge, scan_node, map_server, map_lifecycle, nav_stack])
