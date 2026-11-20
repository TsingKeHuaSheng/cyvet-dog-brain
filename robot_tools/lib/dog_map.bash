# 唯一的地图解析入口（Bash 侧）。解析规则全部在 map_site.py，这里只调用它，不重复实现。
#
# 用法：
#   source ~/robot_tools/lib/dog_map.bash
# 之后可用（都已 export，子进程也能看到）：
#   DOG_ROUTE_MAP   地图名
#   DOG_MAP_DIR     ~/robot_ws/maps/<名字>
#   DOG_MAP_PCD     <dir>/map.pcd            三维定位底图
#   DOG_MAP_GRID    <dir>/nav2/map.yaml      二维栅格（画线底图）
#   DOG_ROUTE_JSON  <dir>/routes/hand_drawn.json  手绘路线
#   DOG_LEGACY_MAP  旧版兜底地图名（仅用于识别没有记录地图的老服务）
#
# 解析顺序：$DOG_ROUTE_MAP → ~/.config/dogroute/selected-map → office4（仅当该目录存在）

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo '请用 source 加载本文件：source ~/robot_tools/lib/dog_map.bash' >&2
    exit 1
fi

# 已经解析过（例如上一步的脚本已经 source 过）就直接返回。
if [ -n "${DOG_MAP_DIR:-}" ] && [ -d "${DOG_MAP_DIR:-}" ]; then
    return 0
fi

_dog_map_py="${DOG_MAP_PY:-$HOME/ws_route/src/office4_route/scripts/map_site.py}"
if [ ! -f "$_dog_map_py" ]; then
    echo "找不到地图解析器：$_dog_map_py" >&2
    return 1
fi

if ! _dog_map_exports=$(/usr/bin/python3 "$_dog_map_py" --shell); then
    echo "无法确定当前地图。先执行 dogroute use <地图名>，或 dogroute status 查看现状。" >&2
    unset _dog_map_py _dog_map_exports
    return 1
fi
eval "$_dog_map_exports"
export DOG_ROUTE_MAP DOG_MAP_DIR DOG_MAP_PCD DOG_MAP_GRID DOG_ROUTE_JSON DOG_LEGACY_MAP
unset _dog_map_py _dog_map_exports

if [ ! -d "$DOG_MAP_DIR" ]; then
    echo "地图目录不存在：$DOG_MAP_DIR（先用 dogroute mapping / save 建这张图）" >&2
    return 1
fi
