import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("localizer"), "rviz", "localizer.rviz"]
    )
    localizer_config_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "config", "localizer.yaml"]
    )

    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )

    geometry_path = Path(get_package_share_directory("localizer")) / "config/base_link_reference.json"
    mount = json.loads(geometry_path.read_text())["lidar_mount_extrinsic"]
    # Both tape measurements start at the mounting face C. FAST-LIO2 body is the IMU.
    base = mount["mount_center_to_base_link_m"]
    imu = mount["mount_center_to_body_imu_m"]
    x, y, z = (base[i] - imu[i] for i in range(3))
    roll, pitch, yaw = mount["base_link_rpy_in_body_rad"]
    body_to_base_link = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_base_link",
        output="screen",
        arguments=[
            "--x", str(x),
            "--y", str(y),
            "--z", str(z),
            "--roll", str(roll),
            "--pitch", str(pitch),
            "--yaw", str(yaw),
            "--frame-id", "body",
            "--child-frame-id", "base_link",
        ],
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[
                    {"config_path": lio_config_path.perform(launch.LaunchContext())}
                ],
            ),
            launch_ros.actions.Node(
                package="localizer",
                namespace="localizer",
                executable="localizer_node",
                name="localizer_node",
                output="screen",
                parameters=[
                    {
                        "config_path": localizer_config_path.perform(
                            launch.LaunchContext()
                        )
                    }
                ],
            ),
            # 固定安装 TF：body -> base_link
            body_to_base_link,
            # 不需要看图时，把下面这个 Node 整段注释掉即可（不影响定位）
            launch_ros.actions.Node(
                package="rviz2",
                namespace="localizer",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            ),
        ]
    )
