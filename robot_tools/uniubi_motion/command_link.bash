#!/usr/bin/env bash
set -e
here=$(dirname "$(realpath "$0")")
source "$here/dog_env.bash"
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="wlP1p1s0"/></Interfaces><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
exec ros2 run domain_bridge domain_bridge --wait-for-publisher false "$here/command_domains.yaml"
