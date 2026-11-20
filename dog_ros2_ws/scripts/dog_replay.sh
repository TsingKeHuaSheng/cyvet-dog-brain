#!/usr/bin/env bash
# ============================================================
# 一条命令回放航点：bridge + follower 一起起来
#
# 用法：
#   ~/dog_ros2_ws/scripts/dog_replay.sh                       # 默认走 first_test.csv
#   ~/dog_ros2_ws/scripts/dog_replay.sh ~/dog_paths/my.csv    # 走自己的路径
#   V_MAX=0.10 ~/dog_ros2_ws/scripts/dog_replay.sh            # 改速度
#
# 运行前必须：遥控器已断开（长按 M 键，听到"遥控器连接已断开"）
# ============================================================
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=dog_env.sh
source "$HERE/dog_env.sh"

mkdir -p "$HOME/dog_paths"

# 路径参数：默认 first_test.csv；没有就从包里拷一份
PATHCSV="${1:-$HOME/dog_paths/first_test.csv}"
if [ ! -f "$PATHCSV" ]; then
    SHARE="$(ros2 pkg prefix dog_waypoint 2>/dev/null || true)/share/dog_waypoint/paths/first_test.csv"
    if [ -f "$SHARE" ]; then
        cp "$SHARE" "$PATHCSV"
        echo "[INFO] 已从包内复制示例路径: $PATHCSV"
    else
        echo "[FAIL] 找不到路径文件: $PATHCSV" >&2
        exit 1
    fi
fi

V_MAX="${V_MAX:-0.12}"
W_MAX="${W_MAX:-0.4}"
MAX_S="${MAX_S:-60}"

echo "=============================================="
echo " 回放航点"
echo "   device_id      = $ROBOT_DEVICE_ID"
echo "   path_file      = $PATHCSV"
echo "   ROS_DOMAIN_ID  = $ROS_DOMAIN_ID"
echo "   CYCLONEDDS_URI = ${CYCLONEDDS_URI:-（未设置，走默认）}"
echo "   v_max          = $V_MAX m/s"
echo "   超时上限       = $MAX_S s"
echo "----------------------------------------------"
echo " 安全确认："
echo "   [ ] 狗在平整空旷地面，四肢周围没人"
echo "   [ ] 遥控器已断开（会听到语音提示）"
echo "   [ ] 急停够得着（遥控器 LB+RB）"
echo "=============================================="
echo ""
echo "3 秒后开始，Ctrl+C 可取消..."
sleep 3

exec ros2 launch dog_waypoint replay.launch.py \
    device_id:="$ROBOT_DEVICE_ID" \
    path_file:="$PATHCSV" \
    v_max:="$V_MAX" \
    w_max:="$W_MAX" \
    max_duration_s:="$MAX_S"
