#!/usr/bin/env bash
# 统一的 ROS 2 环境。被 dog_replay.sh / dog_record.sh source，
# 也可以自己 `source ~/dog_ros2_ws/scripts/dog_env.sh` 后在当前终端用。
#
# 注意：本脚本刻意不使用 set -e，并且所有判断都用 if 而不是 `[ ] && cmd`，
# 这样在被 `set -euo pipefail` 的脚本 source 时，也不会因为某个可选文件
# 不存在而把调用方带崩。

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROBOT_DEVICE_ID="${ROBOT_DEVICE_ID:-E03C1CBAAC40A101}"

# Cyclone DDS 的网络配置：优先尊重环境里已有的设置，否则找几个常见文件名
if [ -z "${CYCLONEDDS_URI:-}" ]; then
    for _f in "$HOME/dds-direct.xml" "$HOME/dds-public.xml" \
              "$HOME/dog_dds/dds-direct.xml" "$HOME/dog_dds/dds-public.xml"; do
        if [ -f "$_f" ]; then
            export CYCLONEDDS_URI="file://$_f"
            break
        fi
    done
    unset _f
fi

if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    . /opt/ros/humble/setup.bash
fi

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    . "$HOME/ros2_ws/install/setup.bash"
fi

if [ -f "$HOME/dog_ros2_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    . "$HOME/dog_ros2_ws/install/setup.bash"
fi
