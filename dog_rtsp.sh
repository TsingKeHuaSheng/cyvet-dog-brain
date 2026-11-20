#!/usr/bin/env bash
# 自动发现机器狗当前 IP 并打印 RTSP 地址
# 用法: ./dog_rtsp.sh [channel]   (默认 channel=1)
set -euo pipefail
CH="${1:-1}"
source "$HOME/env_uniubi.sh"
cd /tmp
IP="$(python3 "$HOME/get_dog_ip.py" 2>/dev/null)"
[ -n "$IP" ] || { echo "未发现机器狗" >&2; exit 1; }
echo "rtsp://${IP}:554/live?channel=${CH}&stream=0"
