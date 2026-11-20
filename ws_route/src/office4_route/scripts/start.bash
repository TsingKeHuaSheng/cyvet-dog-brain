#!/usr/bin/env bash
set -e
source "$(dirname "$(realpath "$0")")/session_guard.bash"
source ~/ws_route/install/setup.bash
export ROS_DOMAIN_ID=0
if ! systemctl --user is-active --quiet office4-route; then
    systemd-run --user --collect --unit=office4-route \
      --setenv=DISPLAY="${DISPLAY:-:0}" --setenv=XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}" \
      /bin/bash -c 'source ~/ws_route/install/setup.bash && export ROS_DOMAIN_ID=0 OPENBLAS_NUM_THREADS=1 && exec ros2 launch office4_route office4.launch.py'
fi
