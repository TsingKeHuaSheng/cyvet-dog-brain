#!/usr/bin/env bash
set -e
here=$(dirname "$(realpath "$0")")
source "$here/dog_env.bash"
exec ros2 run uniubi_motion_bridge uniubi_motion_bridge_node --ros-args \
  --params-file "$here/motion_bridge.yaml" \
  -p robot_service_name:=/robotServer \
  -p auto_connect:=false -p cmd_vel_topic:=/cmd_vel -p cmd_vel_timeout_ms:=450
