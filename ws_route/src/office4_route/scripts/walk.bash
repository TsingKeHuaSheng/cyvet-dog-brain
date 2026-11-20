#!/usr/bin/env bash
set -e
source "$(dirname "$(realpath "$0")")/session_guard.bash"
here=$(dirname "$(realpath "$0")")
test -s "$HOME/robot_ws/maps/${DOG_ROUTE_MAP:-office4}/routes/hand_drawn.json" || { echo 'Draw and save a route in RViz first.'; exit 1; }
systemctl --user start uniubi-motion-bridge
(
    source ~/robot_tools/uniubi_motion/dog_env.bash
    /usr/bin/python3 "$here/dog_ready.py"
)
source ~/ws_route/install/setup.bash
export ROS_DOMAIN_ID=0 OPENBLAS_NUM_THREADS=1
systemctl --user start uniubi-cmd-vel-link
exec /usr/bin/python3 "$here/walk.py" "$@"
