#!/usr/bin/env bash
set -e
here=$(cd -- "$(dirname -- "$0")" && pwd)
start_unit() {
    local unit="$1"
    shift
    if ! systemctl --user is-active --quiet "$unit"; then
        systemd-run --user --collect --unit="$unit" "$@"
    fi
}
systemctl --user start uniubi-motion-bridge.service uniubi-cmd-vel-link.service
start_unit office4-tracker /bin/bash -c 'source "$HOME/ws_slam/install/setup.bash" && export OPENBLAS_NUM_THREADS=1 && exec ros2 launch "$1"' bash "$here/drive.launch.py"
systemctl --user is-active uniubi-motion-bridge uniubi-cmd-vel-link office4-tracker
