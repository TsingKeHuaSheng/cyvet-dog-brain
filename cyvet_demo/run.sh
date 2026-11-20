#!/usr/bin/env bash
# 在 Jetson 上运行本示例，自动带上 SDK 运行库路径和默认网卡/SN。
# 用法：
#   ./run.sh --discover                       只发现设备
#   ./run.sh --read-only                      只读预检（不运动）
#   ./run.sh --yes                            完整动作（站立→前进→转向→趴下）
#   ./run.sh --yes --no-turn --walk-seconds 2 只前进一小段
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1) 环境：source 任务里已有的 env 脚本
if [ -f "$HOME/env_uniubi.sh" ]; then
    # shellcheck disable=SC1090
    source "$HOME/env_uniubi.sh"
fi

# 2) 默认值（env_uniubi.sh 里也导出过这两个变量）
IFACE="${UNIUBI_IFACE:-wlP1p1s0}"
SN="${UNIUBI_DOG_SN:-E03C1CBAAC40A101}"

# 3) 运行库路径：按实际架构补齐（外部主机是 aarch64_host）
SDK_ROOT="${UNIUBI_SDK_ROOT:-$HOME/uniubi_robot_sdk}"
if [ -n "${SDK_ARCH:-}" ]; then
    ARCH="$SDK_ARCH"
elif [ -d "$SDK_ROOT/lib/aarch64_host" ] && [ "$(uname -m)" = "aarch64" ]; then
    ARCH="aarch64_host"
else
    ARCH="$(uname -m)"
fi
export LD_LIBRARY_PATH="$SDK_ROOT/lib/$ARCH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

BIN="$HERE/build/cyvet_dog_demo"
if [ ! -x "$BIN" ]; then
    echo "[FAIL] 还没编译：先跑 ./build.sh" >&2
    exit 1
fi

# 4) --discover 不需要 --device-id；其它模式都带上
if [[ " $* " == *" --discover "* ]]; then
    exec "$BIN" --iface "$IFACE" "$@"
fi

exec "$BIN" --iface "$IFACE" --device-id "$SN" "$@"
