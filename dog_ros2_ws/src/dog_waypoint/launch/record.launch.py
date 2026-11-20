"""一条命令开始录点：同时启动 uniubi_motion_bridge 和 waypoint_recorder。

用法（脚本已封装，见 scripts/dog_record.sh）：
  ros2 launch dog_waypoint record.launch.py \
    device_id:=E03C1CBAAC40A101 \
    path_file:=$HOME/dog_paths/my_path.csv \
    min_step_m:=0.15
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
    min_step_m = LaunchConfiguration("min_step_m")

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

    recorder = Node(
        package="dog_waypoint",
        executable="waypoint_recorder",
        name="waypoint_recorder",
        output="screen",
        parameters=[
            {
                "output_file": ParameterValue(path_file, value_type=str),
                "min_step_m": ParameterValue(min_step_m, value_type=float),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("device_id", default_value="E03C1CBAAC40A101"),
            DeclareLaunchArgument(
                "path_file", default_value="/home/nvidia/dog_paths/my_path.csv"
            ),
            DeclareLaunchArgument("min_step_m", default_value="0.15"),
            bridge,
            recorder,
        ]
    )
