#!/usr/bin/env bash
# 自我定位：把当前所选地图的 map.pcd 交给 localizer 做重定位。
# 只在「该地图的建图起点、与建图时同朝向」调用。地图名来自 dogroute use，本脚本不写死。
set -e
here=$(dirname "$(realpath "$0")")
source "$HOME/robot_tools/lib/dog_map.bash"
source "$here/session_guard.bash"
test -f "$DOG_MAP_PCD" || { echo "$DOG_ROUTE_MAP 还没有 map.pcd：先建图并 dogroute save。" >&2; exit 1; }

source ~/ws_slam/install/setup.bash
export ROS_DOMAIN_ID=0
if ! ros2 service list --no-daemon --spin-time 2 | grep -Fx '/localizer/relocalize' >/dev/null; then
    if ! systemctl --user is-active --quiet office4-localization; then
        systemd-run --user --collect --unit=office4-localization \
          --setenv=DOG_ROUTE_MAP="$DOG_ROUTE_MAP" \
          --setenv=DISPLAY="${DISPLAY:-:0}" --setenv=XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}" \
          /bin/bash -c 'source ~/ws_slam/install/setup.bash && export ROS_DOMAIN_ID=0 && exec ros2 launch localizer localizer_launch.py'
    fi
fi
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '$DOG_MAP_PCD', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
