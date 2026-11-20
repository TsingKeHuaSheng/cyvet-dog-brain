"""一条命令跑通回放：同时启动 uniubi_motion_bridge 和 waypoint_follower。

用法（脚本已封装，见 scripts/dog_replay.sh）：
  ros2 launch dog_waypoint replay.launch.py \
    device_id:=E03C1CBAAC40A101 \
    path_file:=$HOME/dog_paths/first_test.csv \
    v_max:=0.12
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    device_id = LaunchConfiguration("device_id")
    path_file = LaunchConfiguration("path_file")
    v_max = LaunchConfiguration("v_max")
    w_max = LaunchConfiguration("w_max")
    pos_tolerance = LaunchConfiguration("pos_tolerance")
    max_duration_s = LaunchConfiguration("max_duration_s")
    battery_min = LaunchConfiguration("battery_min")
    loop = LaunchConfiguration("loop")

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("uniubi_motion_bridge"),
                    "launch",
                    "motion_bridge.launch.py",
                ]
            )
        ),
        launch_arguments={"device_id": device_id}.items(),
    )

    # 关键：launch 传进来的都是字符串，必须显式声明目标类型，
    # 否则 double/bool 参数会报 "invalid type: expected double, got string"。
    follower = Node(
        package="dog_waypoint",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[
            {
                "path_file": ParameterValue(path_file, value_type=str),
                "v_max": ParameterValue(v_max, value_type=float),
                "w_max": ParameterValue(w_max, value_type=float),
                "pos_tolerance": ParameterValue(pos_tolerance, value_type=float),
                "max_duration_s": ParameterValue(max_duration_s, value_type=float),
                "battery_min": ParameterValue(battery_min, value_type=float),
                "loop": ParameterValue(loop, value_type=bool),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("device_id", default_value="E03C1CBAAC40A101"),
            DeclareLaunchArgument(
                "path_file", default_value="/home/nvidia/dog_paths/first_test.csv"
            ),
            DeclareLaunchArgument("v_max", default_value="0.12"),
            DeclareLaunchArgument("w_max", default_value="0.4"),
            DeclareLaunchArgument("pos_tolerance", default_value="0.15"),
            DeclareLaunchArgument("max_duration_s", default_value="60.0"),
            DeclareLaunchArgument("battery_min", default_value="0.20"),
            DeclareLaunchArgument("loop", default_value="false"),
            bridge,
            follower,
        ]
    )
