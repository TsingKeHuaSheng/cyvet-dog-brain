# 共用守卫：这些服务和话题名被 office4 与之后所有新地图共用。
# 地图名一律来自 dog_map.bash（唯一的解析入口），本文件不写死任何地图名。
source "$HOME/robot_tools/lib/dog_map.bash"

if systemctl --user is-active --quiet dogroute-mapping; then
    echo '正在建图，请先用 dogroute save 保存。' >&2
    exit 1
fi

for route_unit in office4-route office4-localization; do
    if systemctl --user is-active --quiet "$route_unit"; then
        route_loaded=$(systemctl --user show "$route_unit" -p Environment --value | tr ' ' '\n' | sed -n 's/^DOG_ROUTE_MAP=//p')
        if [ -z "$route_loaded" ]; then
            # 旧方式启动（改版前）的服务不记录地图，按遗留地图处理并提示。
            route_loaded="$DOG_LEGACY_MAP"
            echo "[提示] $route_unit 是旧方式启动的（没记录地图），按 $DOG_LEGACY_MAP 处理。建议 dogroute stop 后用 dogroute locate / draw 重新启动。" >&2
        fi
        if [ "$route_loaded" != "$DOG_ROUTE_MAP" ]; then
            echo "$route_unit 正在使用 $route_loaded，而当前选择是 $DOG_ROUTE_MAP。请先用 dogroute stop 关闭旧现场（或执行 dogroute use $route_loaded 切回去）。" >&2
            exit 1
        fi
    fi
done
