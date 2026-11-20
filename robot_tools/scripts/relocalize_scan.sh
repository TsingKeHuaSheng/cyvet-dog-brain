#!/bin/bash
# ============================================================
# 重定位初值网格扫描 (relocalize scan)
# 用法: ./relocalize_scan.sh [地图pcd] [x] [y] [z]
# 默认: run3 地图, 位置全零 (假设在地图原点附近)
#
# 前提 (来自 ~/robot_tools/docs/LOCALIZATION.txt):
#   1. lio_node + localizer_node 已启动 (localizer_launch.py)
#   2. 雷达静止, 位置在地图原点附近
#   3. 本脚本会枚举 8 个朝向初值, 收敛即停
#
# 注意: 收敛成功后不要再调 /localizer/relocalize,
#       每次 relocalize 调用都会把 localize_success 清为 false!
# ============================================================
set -u
source "$HOME/robot_tools/lib/dog_map.bash" || exit 1
MAP=${1:-${DOG_MAP_PCD:-}}
[ -n "$MAP" ] || { echo "没有选定地图：先 dogroute use <名字>" >&2; exit 1; }
X=${2:-0.0}
Y=${3:-0.0}
Z=${4:-0.0}

source /opt/ros/humble/setup.bash
source /home/nvidia/ws_livox/install/setup.bash
source /home/nvidia/ws_slam/install/setup.bash

if [ ! -f "$MAP" ]; then
    echo "ERROR: 地图不存在: $MAP"
    echo "可用地图:"
    ls ~/robot_ws/maps/*/map.pcd 2>/dev/null
    exit 1
fi

echo "地图: $MAP"
echo "位置初值: x=$X y=$Y z=$Z"
echo "开始朝向扫描 (8 个方向)..."
echo ""

for yaw in 0.0 0.7854 -0.7854 1.5708 -1.5708 2.3562 -2.3562 3.1416; do
    deg=$(python3 -c "print(int(float('$yaw')*57.2958))")
    echo "== 尝试 yaw=${deg}deg =="
    timeout 30 ros2 service call /localizer/relocalize interface/srv/Relocalize \
        "{pcd_path: '$MAP', x: $X, y: $Y, z: $Z, yaw: $yaw, pitch: 0.0, roll: 0.0}" \
        >/dev/null 2>&1 || { echo "   服务调用失败"; continue; }
    sleep 5
    valid=$(timeout 15 ros2 service call /localizer/relocalize_check interface/srv/IsValid "{code: 0}" 2>&1 | grep -c "valid=True")
    if [ "$valid" -ge 1 ]; then
        echo ""
        echo "=============================================="
        echo "  定位成功! 初值: yaw=${deg}deg, x=$X y=$Y z=$Z"
        echo "  TF map->lidar 已在发布, 不要再调 relocalize"
        echo "  查看状态: ros2 service call /localizer/relocalize_check interface/srv/IsValid '{code: 0}'"
        echo "=============================================="
        exit 0
    fi
done

echo ""
echo "全部 8 个初值未收敛。排查建议:"
echo "  1. 雷达是否还在地图覆盖范围内? (run3 行程约 7.5m)"
echo "  2. 位置不在原点? 试: $0 $MAP <x> <y> <z>  传入 map 系下的大致位置"
echo "  3. 地图太久/环境变化大? 重新建图 (见 docs/MAPPING.txt)"
echo "  4. lio_node 是否刚重启过? 世界系已变, 需要物理摆位后再扫"
exit 2
