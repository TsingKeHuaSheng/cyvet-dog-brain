import json
import os
from pathlib import Path
from tempfile import NamedTemporaryFile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def grid_view(template, name):
    view = yaml.safe_load(template.read_text())
    displays = view['Visualization Manager']['Displays']
    background = next(item for item in displays if item.get('Topic', {}).get('Value') == '/line_route/office4')
    background.update(Name='3D map', Enabled=False, Value=False)
    background['Topic']['Value'] = '/line_route/pointcloud'
    displays.insert(0, {'Class': 'rviz_default_plugins/Map', 'Name': f'{name} / Nav2 grid',
                        'Enabled': True, 'Value': True, 'Draw Behind': True, 'Alpha': 0.8,
                        'Topic': {'Value': '/line_route/map', 'Reliability Policy': 'Reliable',
                                  'Durability Policy': 'Transient Local', 'Depth': 1}})
    folder = Path.home() / '.cache/dogroute'
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f'{name}_Draw.rviz'
    with NamedTemporaryFile('w', dir=folder, delete=False) as stream:
        yaml.safe_dump(view, stream, sort_keys=False, allow_unicode=True)
    os.replace(stream.name, path)
    return path


def generate_launch_description():
    cfg = Path(__file__).resolve().parent
    scripts = cfg.parent / 'scripts'
    settings = str(cfg / 'controller.yaml')
    rviz = cfg / 'Office4_Draw.rviz'
    floor_args = []
    servers = ['controller_server']
    nodes = []
    name = os.environ.get('DOG_ROUTE_MAP')
    if name:
        directory = Path.home() / 'robot_ws/maps' / name / 'nav2'
        floor = float(json.loads((directory / 'projection.json').read_text())['floor_z'])
        floor_args = ['-p', f'floor_z:={floor}']
        rviz = grid_view(rviz, name)
        servers.insert(0, 'map_server')
        nodes.append(Node(package='nav2_map_server', executable='map_server', name='map_server',
                          namespace='line_route', parameters=[{'yaml_filename': str(directory / 'map.yaml'),
                                                               'frame_id': 'map', 'topic_name': 'map'}]))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        ExecuteProcess(cmd=['/usr/bin/python3', str(scripts / 'editor.py'), '--ros-args', *floor_args]),
        ExecuteProcess(cmd=['/usr/bin/python3', str(scripts / 'obstacles.py'),
                            '--ros-args', '--params-file', settings, *floor_args]),
        ExecuteProcess(cmd=['/usr/bin/python3', str(Path.home() / 'robot_tools/office4_drive/base_odom.py')]),
        *nodes,
        Node(package='nav2_controller', executable='controller_server', namespace='line_route',
             parameters=[settings], remappings=[('cmd_vel', '/line_route/tracking')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle',
             namespace='line_route', parameters=[{'autostart': True, 'node_names': servers}]),
        Node(package='rviz2', executable='rviz2', name='office4_draw', arguments=['-d', str(rviz)],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
