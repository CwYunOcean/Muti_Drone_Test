#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$BASE_REPO_ROOT/scripts/load_robot_config.sh"

OVERLAY_WS="$BASE_REPO_ROOT/overlay_ws"

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$OVERLAY_WS/install/setup.bash"
set -u

export RMW_IMPLEMENTATION
USE_ACCELERATION_FF="${USE_ACCELERATION_FF:-true}"

cd "$OVERLAY_WS"

echo "Starting single-drone GVF/ISMC real-hardware bringup..."
echo "drone_id=$DRONE_ID"
echo "use_acceleration_feedforward=$USE_ACCELERATION_FF"
echo "For experiment logging, open another terminal and run:"
echo "  ./scripts/record_gvf_ismc_experiment.sh"
ros2 launch gvf_ismc_real_bringup single_real.launch.py \
  drone_id:="$DRONE_ID" \
  use_acceleration_feedforward:="$USE_ACCELERATION_FF" &
LAUNCH_PID=$!

# rviz2 -d "$OVERLAY_WS/src/gvf_ismc_real_bringup/rviz/gvf_ismc_single_real.rviz"
wait "$LAUNCH_PID"
