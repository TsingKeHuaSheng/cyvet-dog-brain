#!/usr/bin/env bash
# 用法：start_preview.sh [map.yaml 路径]
# 不传参数时默认用 office2 的栅格
set -e
source /opt/ros/humble/setup.bash
PREVIEW_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MAP_YAML="${1:-/home/nvidia/robot_ws/maps/office2/navigation_preview_v2/map.yaml}"

if [ ! -f "$MAP_YAML" ]; then
    echo "❌ 找不到地图: $MAP_YAML"
    echo "现成的："
    ls -1 /home/nvidia/robot_ws/maps/*/navigation_preview*/map.yaml 2>/dev/null | sed 's/^/   /'
    exit 1
fi

echo "使用地图: $MAP_YAML"
exec ros2 launch "$PREVIEW_DIR/planning_preview.launch.py" map_yaml:="$MAP_YAML"
