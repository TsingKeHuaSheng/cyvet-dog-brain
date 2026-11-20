# These service and topic names are shared by legacy Office4 and all new maps.
if systemctl --user is-active --quiet dogroute-mapping; then
    echo '正在建图，请先用 dogroute save 保存。' >&2
    exit 1
fi
for route_unit in office4-route office4-localization; do
    if systemctl --user is-active --quiet "$route_unit"; then
        route_loaded=$(systemctl --user show "$route_unit" -p Environment --value | tr ' ' '\n' | sed -n 's/^DOG_ROUTE_MAP=//p')
        if [[ "${route_loaded:-office4}" != "${DOG_ROUTE_MAP:-office4}" ]]; then
            echo "$route_unit 正在使用 ${route_loaded:-office4}。请用 dogroute 切图，或先用 dogroute stop 关闭旧现场。" >&2
            exit 1
        fi
    fi
done
