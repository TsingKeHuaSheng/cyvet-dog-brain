from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parent
    config = str(root / 'controller.yaml')
    ns = 'office4_drive'
    return LaunchDescription([
        # FAST-LIO's configured T_imu_lidar, mapping raw LiDAR points into body.
        Node(package='tf2_ros', executable='static_transform_publisher', namespace=ns,
             arguments=['--frame-id','body','--child-frame-id','livox_frame',
                        '--x','-0.011','--y','-0.02329','--z','0.04412']),
        Node(package='livox_pc2_bridge', executable='livox_pc2_bridge_node', namespace=ns,
             remappings=[('/livox/lidar_pc2','/office4_drive/raw_cloud')]),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node', namespace=ns,
             parameters=[{'min_height':-0.30, 'max_height':0.36, 'range_min':0.12,
                          'range_max':3.0,'angle_increment':0.0174533,'concurrency_level':1}],
             remappings=[('cloud_in','/office4_drive/raw_cloud'),('scan','/office4_drive/near_scan')]),
        ExecuteProcess(cmd=['/usr/bin/python3', str(root / 'base_odom.py')]),
        Node(package='nav2_controller', executable='controller_server', namespace=ns,
             parameters=[config], remappings=[('cmd_vel','/office4_drive/tracking_velocity')]),
        Node(package='nav2_collision_monitor', executable='collision_monitor', namespace=ns,
             name='collision_monitor', parameters=[config]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', namespace=ns,
             name='lifecycle', parameters=[{'node_names':['controller_server','collision_monitor'],
                                            'autostart':True}]),
    ])
