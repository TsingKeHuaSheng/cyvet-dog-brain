#!/usr/bin/env bash
# ============================================================
# 环境体检：确认包、话题、/odom 数据、服务都在
# 用法： ~/dog_ros2_ws/scripts/dog_check.sh
# 建议在 bridge 已启动的另一个终端里跑
# ============================================================
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=dog_env.sh
source "$HERE/dog_env.sh"

echo "ROS_DOMAIN_ID      = $ROS_DOMAIN_ID"
echo "CYCLONEDDS_URI     = ${CYCLONEDDS_URI:-（未设置）}"
echo "ROS_LOCALHOST_ONLY = $ROS_LOCALHOST_ONLY"
echo ""

echo "=== 包 ==="
if ros2 pkg prefix dog_waypoint >/dev/null 2>&1; then
    echo "  OK   dog_waypoint"
else
    echo "  MISS dog_waypoint（先 colcon build + source install/setup.bash）"
fi
if ros2 pkg prefix uniubi_motion_bridge >/dev/null 2>&1; then
    echo "  OK   uniubi_motion_bridge"
else
    echo "  MISS uniubi_motion_bridge（source ~/ros2_ws/install/setup.bash）"
fi
echo ""

echo "=== 关键话题 ==="
TOPICS="$(ros2 topic list 2>/dev/null)"
for t in /cmd_vel /odom /motion/status /battery_state; do
    if printf '%s\n' "$TOPICS" | grep -qx "$t"; then
        echo "  OK   $t"
    else
        echo "  MISS $t"
    fi
done
echo ""

# /odom 是 Best Effort，订阅端必须一致，否则一条也收不到
echo "=== /odom 采样（3 秒，best_effort）==="
if timeout 3 ros2 topic echo /odom --once --qos-reliability best_effort >/dev/null 2>&1; then
    echo "  有数据"
else
    echo "  没收到 /odom —— 狗必须处于 Walk 动作期间（先 /motion/start_action）"
fi
echo ""

echo "=== /motion/status 采样 ==="
if timeout 3 ros2 topic echo /motion/status --once >/dev/null 2>&1; then
    echo "  有数据"
else
    echo "  没收到 /motion/status —— bridge 起来了吗？"
fi
echo ""

echo "=== /motion/* 服务 ==="
ros2 service list 2>/dev/null | grep '/motion/' | sort
