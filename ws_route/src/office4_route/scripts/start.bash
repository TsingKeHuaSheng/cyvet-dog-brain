#!/usr/bin/env bash
# 打开当前所选地图的画线界面（底图 = 该地图的二维栅格）。
# 地图名来自 dogroute use，本脚本不写死；服务名 office4-route 只是历史命名，与地图名无关。
set -e
here=$(dirname "$(realpath "$0")")
source "$HOME/robot_tools/lib/dog_map.bash"
source "$here/session_guard.bash"
test -f "$DOG_MAP_GRID" || { echo "$DOG_ROUTE_MAP 还没有二维栅格：先执行 dogroute grid。" >&2; exit 1; }

source ~/ws_route/install/setup.bash
export ROS_DOMAIN_ID=0
if ! systemctl --user is-active --quiet office4-route; then
    systemd-run --user --collect --unit=office4-route \
      --setenv=DOG_ROUTE_MAP="$DOG_ROUTE_MAP" \
      --setenv=DISPLAY="${DISPLAY:-:0}" --setenv=XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}" \
      /bin/bash -c 'source ~/ws_route/install/setup.bash && export ROS_DOMAIN_ID=0 OPENBLAS_NUM_THREADS=1 && exec ros2 launch office4_route office4.launch.py'
fi
