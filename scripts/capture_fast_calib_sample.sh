#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OVERLAY_WS="$BASE_REPO_ROOT/overlay_ws"

DEPENDENCY_REPO_ROOT="$BASE_REPO_ROOT"
SLAM_SETUP_FILE="$DEPENDENCY_REPO_ROOT/slam_ws/install/setup.bash"
LIVOX_CONFIG_FILE="${LIVOX_CONFIG_FILE:-$OVERLAY_WS/src/fast_lio2_mid360_bringup/config/MID360_config_drone_1.json}"
if ([[ ! -f "$SLAM_SETUP_FILE" ]] || [[ ! -f "$LIVOX_CONFIG_FILE" ]]) && [[ "$BASE_REPO_ROOT" == */.worktrees/* ]]; then
  DEPENDENCY_REPO_ROOT="${BASE_REPO_ROOT%%/.worktrees/*}"
  SLAM_SETUP_FILE="$DEPENDENCY_REPO_ROOT/slam_ws/install/setup.bash"
  LIVOX_CONFIG_FILE="${LIVOX_CONFIG_FILE:-$DEPENDENCY_REPO_ROOT/overlay_ws/src/fast_lio2_mid360_bringup/config/MID360_config_drone_1.json}"
fi
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

BAG_BASE_DIR="${FAST_CALIB_OUTPUT_DIR:-/tmp/fast_calib}"
BAG_PATH="$BAG_BASE_DIR/calibration_sample"

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

cd "$OVERLAY_WS"
colcon build --symlink-install --packages-select fast_livo2_mid360_bringup
set +u
source "$OVERLAY_WS/install/setup.bash"
set -u

mkdir -p "$BAG_BASE_DIR"

echo "Starting MID360 + D435i calibration capture stack..."
ros2 launch fast_livo2_mid360_bringup mid360_d435i_calib_capture.launch.py \
  livox_config_file:="$LIVOX_CONFIG_FILE" &
LAUNCH_PID=$!

sleep 3

echo "Recording calibration bag to $BAG_PATH"
ros2 bag record -o "$BAG_PATH" \
  /livox/lidar \
  /camera/camera/infra1/image_rect_raw
