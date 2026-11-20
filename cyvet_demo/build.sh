#!/usr/bin/env bash
# 在 Jetson（外部 ARM64 主机）上编译本示例。
# 用法：  ./build.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

# 1) 复用任务里已有的环境脚本（里面有 UNIUBI_SDK_ROOT 和 LD_LIBRARY_PATH）
if [ -z "${UNIUBI_SDK_ROOT:-}" ] && [ -f "$HOME/env_uniubi.sh" ]; then
    # shellcheck disable=SC1090
    source "$HOME/env_uniubi.sh"
fi
SDK_ROOT="${UNIUBI_SDK_ROOT:-$HOME/uniubi_robot_sdk}"
export UNIUBI_SDK_ROOT="$SDK_ROOT"

if [ ! -d "$SDK_ROOT/include/uniubi" ]; then
    echo "[FAIL] 找不到 SDK 头文件：$SDK_ROOT/include/uniubi" >&2
    echo "       改一下 env_uniubi.sh 里的路径，或：UNIUBI_SDK_ROOT=/path/to/sdk ./build.sh" >&2
    exit 1
fi

# 2) 编译。PLATFORM=aarch64_host 是关键：我们是外部主机，不是狗的大脑板
cmake -S . -B build \
    -DPLATFORM=aarch64_host \
    -DUNIUBI_SDK_ROOT="$SDK_ROOT" \
    -DCMAKE_BUILD_TYPE=Release

cmake --build build -j"$(nproc)"

echo
echo "[PASS] 编译完成：$HERE/build/cyvet_dog_demo"
file "$HERE/build/cyvet_dog_demo" 2>/dev/null || true
