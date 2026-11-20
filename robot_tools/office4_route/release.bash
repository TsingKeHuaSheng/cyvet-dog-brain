#!/usr/bin/env bash
set -e
source ~/robot_tools/uniubi_motion/dog_env.bash
ros2 service call /motion/release_control std_srvs/srv/Trigger '{}'
ros2 topic echo /motion/status --once --field control_state 2>/dev/null || true
