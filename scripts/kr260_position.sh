#!/usr/bin/env bash
# Manual entry: no automatic arming, flight commands, or boot activation.
set -eo pipefail
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-status}" == "_run" ]]; then
  [[ -n "${INVOCATION_ID:-}" ]] || { echo "_run is reserved for systemd" >&2; exit 1; }
  source /opt/ros/humble/setup.bash
  source "$WORKSPACE/install/setup.bash"
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0
  cd "$WORKSPACE"
  exec ros2 launch top_launch_pkg kr260_position.launch.py
fi
RUNTIME="$WORKSPACE/install/top_launch_pkg/lib/top_launch_pkg/kr260_position_runtime.py"
[[ -f "$RUNTIME" ]] || { echo "Not installed: deploy/position_onekey/README.md" >&2; exit 1; }
exec /usr/bin/python3 "$RUNTIME" "${@:-status}" --workspace "$WORKSPACE"
