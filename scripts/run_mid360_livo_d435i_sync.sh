#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$BASE_REPO_ROOT/scripts/load_robot_config.sh"

OVERLAY_WS="$BASE_REPO_ROOT/overlay_ws"
SLAM_WS="$BASE_REPO_ROOT/slam_ws"
RVIZ_CONFIG_FILE="$BASE_REPO_ROOT/overlay_ws/src/fast_livo2_mid360_bringup/rviz/fast_livo2_livo_d435i.rviz"

DEPENDENCY_REPO_ROOT="$BASE_REPO_ROOT"
SLAM_SETUP_FILE="$SLAM_WS/install/setup.bash"
LIVOX_CONFIG_FILE="${LIVOX_CONFIG_FILE:-$OVERLAY_WS/src/fast_lio2_mid360_bringup/config/MID360_config_drone_1.json}"

if ([[ ! -f "$SLAM_SETUP_FILE" ]] || [[ ! -f "$LIVOX_CONFIG_FILE" ]]) && [[ "$BASE_REPO_ROOT" == */.worktrees/* ]]; then
  DEPENDENCY_REPO_ROOT="${BASE_REPO_ROOT%%/.worktrees/*}"
  SLAM_SETUP_FILE="$DEPENDENCY_REPO_ROOT/slam_ws/install/setup.bash"
  LIVOX_CONFIG_FILE="${LIVOX_CONFIG_FILE:-$DEPENDENCY_REPO_ROOT/overlay_ws/src/fast_lio2_mid360_bringup/config/MID360_config_drone_1.json}"
fi

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

if [[ ! -f "$LIVOX_CONFIG_FILE" ]]; then
  echo "Livox config does not exist: $LIVOX_CONFIG_FILE" >&2
  exit 1
fi

set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$SLAM_SETUP_FILE"
set -u

cd "$OVERLAY_WS"
colcon build --symlink-install --packages-select fast_livo2_mid360_bringup
set +u
source "$OVERLAY_WS/install/setup.bash"
set -u

echo "Starting MID360 synchronized LIVO + D435i stack..."
ros2 launch fast_livo2_mid360_bringup mid360_livo_d435i_sync_experiment.launch.py \
  livox_config_file:="$LIVOX_CONFIG_FILE" \
  infra_profile:="640x480x10" &
LAUNCH_PID=$!

sleep 3

echo "Opening RViz..."
rviz2 -d "$RVIZ_CONFIG_FILE"
