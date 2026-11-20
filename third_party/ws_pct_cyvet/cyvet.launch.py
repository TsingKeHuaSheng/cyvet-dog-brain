"""带箱灵猫的 PCT 预览。地图目录和场景配置同名。"""
from math import hypot, isfinite
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from global_planner.resources import cache_dir, share_dir


def configure(context):
    share = share_dir()
    name = LaunchConfiguration('map_name').perform(context)
    if not name or name in ('.', '..') or Path(name).name != name:
        raise ValueError('map_name 应为地图目录的名称')
    scene = yaml.safe_load((share / 'config/maps' / (name + '.yaml')).read_text())
    robot = yaml.safe_load((share / 'config/cyvet.yaml').read_text())
    folder = Path.home() / 'robot_ws/maps' / name
    pcd = folder / 'map.pcd'
    tomo = folder / 'pct' / (name + '.pickle')
    if not pcd.is_file():
        raise FileNotFoundError(pcd)
    if LaunchConfiguration('mode').perform(context) == 'build':
        positive = [robot[k] for k in ('length', 'width', 'height', 'step_max')]
        positive.extend([scene['slice_dh'], scene['resolution']])
        if not all(isfinite(v) and v > 0 for v in positive) or not isfinite(scene['ground_h']):
            raise ValueError('车体尺寸、分辨率、层间距和高差阈值须为正数，地面高度须为有限值')
        minimum = robot['min_clearance_margin']
        preferred = robot['preferred_clearance_margin']
        if not (0 <= minimum <= preferred and isfinite(preferred)):
            raise ValueError('净空余量须满足 0 <= 最小余量 <= 优选余量')
        return [
            SetEnvironmentVariable('CUDA_PATH', '/usr/local/cuda'),
            SetEnvironmentVariable('CUPY_CACHE_DIR', str(cache_dir() / 'cupy')),
            Node(package='global_planner', executable='tomography_node', output='screen',
                 parameters=[{
                     'pcd_file': str(pcd), 'tomogram_file': str(tomo),
                     'ground_h': float(scene['ground_h']), 'slice_dh': float(scene['slice_dh']),
                     'resolution': float(scene['resolution']), 'step_max': float(robot['step_max']),
                     'interval_min': robot['height'] + minimum,
                     'interval_free': robot['height'] + preferred,
                     'inflation': hypot(robot['length'], robot['width']) / 2,
                 }]),
        ]
    if not tomo.is_file():
        raise FileNotFoundError(f'{tomo} 尚未生成，请先运行 mode:=build')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/plan.launch.py')),
                                 launch_arguments={
                                     'pcd_file': str(pcd), 'tomogram_file': str(tomo),
                                     'start_frame_id': robot['base_frame'], 'use_sim_time': 'false',
                                     'wait_for_goal': 'true',
                                 }.items()),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'rsc/rviz/pct_ros.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='office4'),
        DeclareLaunchArgument('mode', default_value='plan', choices=['build', 'plan']),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=configure),
    ])
