#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$BASE_REPO_ROOT/scripts/load_robot_config.sh"

OVERLAY_WS="$BASE_REPO_ROOT/overlay_ws"
RVIZ_CONFIG_FILE="$BASE_REPO_ROOT/slam_ws/src/FAST_LIO/rviz/fastlio.rviz"
OPEN_RVIZ="${OPEN_RVIZ:-auto}"

DEPENDENCY_REPO_ROOT="$BASE_REPO_ROOT"
if [[ ! -f "$DEPENDENCY_REPO_ROOT/slam_ws/install/setup.bash" ]] && [[ "$BASE_REPO_ROOT" == */.worktrees/* ]]; then
  DEPENDENCY_REPO_ROOT="${BASE_REPO_ROOT%%/.worktrees/*}"
fi

SLAM_SETUP_FILE="$DEPENDENCY_REPO_ROOT/slam_ws/install/setup.bash"
LIVOX_CONFIG_FILE="${LIVOX_CONFIG_FILE:-$OVERLAY_WS/src/fast_lio2_mid360_bringup/config/MID360_config_drone_1.json}"

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$SLAM_SETUP_FILE"
set -u

export RMW_IMPLEMENTATION

if [[ ! -f "$LIVOX_CONFIG_FILE" ]]; then
  echo "Livox config does not exist: $LIVOX_CONFIG_FILE" >&2
  exit 1
fi

cd "$OVERLAY_WS"
colcon build --symlink-install --packages-select fast_lio2_mid360_bringup
set +u
source "$OVERLAY_WS/install/setup.bash"
set -u

echo "Starting MID360 FAST-LIO2 stack..."
ros2 launch fast_lio2_mid360_bringup mid360_fastlio2.launch.py \
  livox_config_file:="$LIVOX_CONFIG_FILE" \
  drone_id:="$DRONE_ID" &
LAUNCH_PID=$!

sleep 3

if [[ "$OPEN_RVIZ" == "true" ]] || {
  [[ "$OPEN_RVIZ" == "auto" ]] &&
    { [[ -n "${DISPLAY:-}" ]] || [[ -n "${WAYLAND_DISPLAY:-}" ]]; }
}; then
  echo "Opening RViz with FAST-LIO2 config..."
  rviz2 -d "$RVIZ_CONFIG_FILE"
else
  echo "RViz disabled or no graphical display detected. FAST-LIO2 keeps running."
fi

wait "$LAUNCH_PID"
