#!/usr/bin/env bash
# 循线行走：地图来自 dogroute use 的选择，本脚本不含任何地图名。
set -e
here=$(dirname "$(realpath "$0")")
source "$HOME/robot_tools/lib/dog_map.bash"
source "$here/session_guard.bash"
test -s "$DOG_ROUTE_JSON" || { echo "还没有为 $DOG_ROUTE_MAP 画线：先 dogroute draw，在 RViz 里画完松手保存。" >&2; exit 1; }
systemctl --user start uniubi-motion-bridge
(
    source ~/robot_tools/uniubi_motion/dog_env.bash
    /usr/bin/python3 "$here/dog_ready.py"
)
source ~/ws_route/install/setup.bash
export ROS_DOMAIN_ID=0 OPENBLAS_NUM_THREADS=1
systemctl --user start uniubi-cmd-vel-link
exec /usr/bin/python3 "$here/walk.py" "$@"
