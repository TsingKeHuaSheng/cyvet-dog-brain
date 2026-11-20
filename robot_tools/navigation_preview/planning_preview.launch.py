from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = Path(__file__).resolve().parent
    map_yaml = LaunchConfiguration('map_yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', description='Occupancy grid YAML from the same map used by Localizer'),
        Node(package='nav2_map_server', executable='map_server', name='preview_map_server',
             parameters=[{'use_sim_time': False, 'yaml_filename': map_yaml,
                          'frame_id': 'map', 'topic_name': '/navigation_preview/map'}], output='screen'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[str(here / 'planner.yaml')], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='preview_lifecycle_manager', parameters=[{
                 'use_sim_time': False, 'autostart': True,
                 'node_names': ['preview_map_server', 'planner_server']}], output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(here / 'goal_to_plan.py')], output='screen'),
    ])
