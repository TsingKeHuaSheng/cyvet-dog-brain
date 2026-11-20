#!/usr/bin/env bash
# ============================================================
#  一键查看 Livox MID-360S 原始点云
#  用法：~/view_raw_cloud.sh
#  退出：在 RViz 里关闭窗口，或在终端按 Ctrl+C
# ============================================================
# （不用 set -u：ROS 的 setup.bash 会用未定义变量）

# ---------- 环境（脚本自己 source，不依赖 .bashrc）----------
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"

# ---------- 图形环境（允许从 SSH 启动）----------
export DISPLAY="${DISPLAY:-:0}"
if [ -z "${XAUTHORITY:-}" ] && [ -f "/run/user/$(id -u)/gdm/Xauthority" ]; then
    export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
fi

BRIDGE_PID=""
cleanup() {
    if [ -n "$BRIDGE_PID" ] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
        echo
        echo "清理：关闭转换节点 (pid $BRIDGE_PID)"
        kill "$BRIDGE_PID" 2>/dev/null
    fi
}
trap cleanup EXIT INT TERM

echo "========================================"
echo " 查看 Livox 原始点云"
echo "========================================"

# ---------- 1. 检查雷达驱动 ----------
echo "[1/3] 检查雷达驱动…"
if ! systemctl is-active --quiet livox-lidar; then
    echo "  驱动没在跑，尝试启动…"
    sudo systemctl start livox-lidar || { echo "  ❌ 启动失败"; exit 1; }
    sleep 6
fi
if timeout 8 ros2 topic echo --once /livox/lidar --field point_num >/dev/null 2>&1; then
    echo "  ✅ 雷达正常（/livox/lidar 有数据）"
else
    echo "  ❌ /livox/lidar 没有数据"
    echo "     检查：雷达是否通电、网线是否插好、ping 192.168.1.194"
    echo "     看日志：journalctl -u livox-lidar -n 30"
    exit 1
fi

# ---------- 2. 启动转换节点 ----------
echo "[2/3] 启动 CustomMsg → PointCloud2 转换…"
if pgrep -f livox_pc2_bridge_node >/dev/null 2>&1; then
    echo "  已在运行，直接复用"
else
    nohup ros2 run livox_pc2_bridge livox_pc2_bridge_node \
        > /tmp/livox_pc2_bridge.log 2>&1 &
    BRIDGE_PID=$!
    sleep 5
    if kill -0 "$BRIDGE_PID" 2>/dev/null; then
        echo "  ✅ 已启动 (pid $BRIDGE_PID)"
    else
        echo "  ❌ 启动失败，看 /tmp/livox_pc2_bridge.log"
        exit 1
    fi
fi

# ---------- 3. 打开 RViz ----------
echo "[3/3] 打开 RViz…"
echo "  （关闭 RViz 窗口即可退出）"
echo "----------------------------------------"

rviz2 -d "$HOME/robot_tools/rviz/rviz_livox_pc2.rviz"

echo "RViz 已关闭。"
