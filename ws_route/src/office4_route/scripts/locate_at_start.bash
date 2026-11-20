#!/usr/bin/env bash
# Invoke only when the robot is at the Office4 mapping origin in the original orientation.
set -e
source "$(dirname "$(realpath "$0")")/session_guard.bash"
source ~/ws_slam/install/setup.bash
export ROS_DOMAIN_ID=0
if ! ros2 service list --no-daemon --spin-time 2 | grep -Fx '/localizer/relocalize' >/dev/null; then
    if ! systemctl --user is-active --quiet office4-localization; then
        systemd-run --user --collect --unit=office4-localization \
          --setenv=DISPLAY="${DISPLAY:-:0}" --setenv=XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}" \
          /bin/bash -c 'source ~/ws_slam/install/setup.bash && export ROS_DOMAIN_ID=0 && exec ros2 launch localizer localizer_launch.py'
    fi
fi
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '$HOME/robot_ws/maps/office4/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
