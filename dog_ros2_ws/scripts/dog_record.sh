#!/usr/bin/env bash
# ============================================================
# 一条命令开始录点：bridge + recorder 一起起来
#
# 用法：
#   ~/dog_ros2_ws/scripts/dog_record.sh                    # 存到 ~/dog_paths/recorded.csv
#   ~/dog_ros2_ws/scripts/dog_record.sh ~/dog_paths/p1.csv
#
# 起来之后，**另开一个终端**用下面这条驱动狗（这就是"第 2 条命令"）：
#
#   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.12}}'
#   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.25}}'
#   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{}'      # 停
#
# 走完路线后：先在驱动终端 Ctrl+C，再回到本终端 Ctrl+C（CSV 已落盘）
# ============================================================
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=dog_env.sh
source "$HERE/dog_env.sh"

mkdir -p "$HOME/dog_paths"
OUT="${1:-$HOME/dog_paths/recorded.csv}"

echo "=============================================="
echo " 录点"
echo "   device_id      = $ROBOT_DEVICE_ID"
echo "   输出文件       = $OUT"
echo "   ROS_DOMAIN_ID  = $ROS_DOMAIN_ID"
echo "   CYCLONEDDS_URI = ${CYCLONEDDS_URI:-（未设置，走默认）}"
echo "----------------------------------------------"
echo " 起来后另开一个终端，先取权再驱动："
echo ""
echo "   ros2 service call /motion/start_action \\"
echo "     uniubi_motion_bridge/srv/StartMotionAction \\"
echo "     \"{action: walking, params_json: '{\\\"lineVelocityX\\\":0.0,\\\"lineVelocityY\\\":0.0,\\\"velocity\\\":0.0}'}\""
echo ""
echo "   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.12}}'"
echo ""
echo " 走完 Ctrl+C 本终端即可，CSV 每采一点就落盘。"
echo "=============================================="
echo ""
echo "3 秒后启动，Ctrl+C 可取消..."
sleep 3

exec ros2 launch dog_waypoint record.launch.py \
    device_id:="$ROBOT_DEVICE_ID" \
    path_file:="$OUT"
