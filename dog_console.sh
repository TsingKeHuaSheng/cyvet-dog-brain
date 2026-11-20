#!/usr/bin/env bash
# 宇泛灵猫 Cyvet 机器狗 —— High-level 控制台（Jetson 侧）
# 用法: ~/dog_console.sh            进交互控制台
#       ~/dog_console.sh --discover-only   只发现，不连接
set -euo pipefail
source "$HOME/env_uniubi.sh"
cd /tmp   # 必须离开 SDK 源码目录，避免加载未编译的 robot_motion_sdk
exec "$HOME/venvs/uniubi/bin/python" \
    "$HOME/uniubi_robot_sdk_py/examples/example_highlevel.py" \
    --iface "${UNIUBI_IFACE:-wlP1p1s0}" \
    --device-id "${UNIUBI_DOG_SN:-E03C1CBAAC40A101}" "$@"
