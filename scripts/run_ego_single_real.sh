#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$BASE_REPO_ROOT/scripts/load_robot_config.sh"

OVERLAY_WS="$BASE_REPO_ROOT/overlay_ws"

DEPENDENCY_REPO_ROOT="$BASE_REPO_ROOT"
if [[ ! -f "$DEPENDENCY_REPO_ROOT/nav_ws/install/setup.bash" ]] && [[ "$BASE_REPO_ROOT" == */.worktrees/* ]]; then
  DEPENDENCY_REPO_ROOT="${BASE_REPO_ROOT%%/.worktrees/*}"
fi

NAV_SETUP_FILE="$DEPENDENCY_REPO_ROOT/nav_ws/install/setup.bash"
RVIZ_CONFIG_FILE="$BASE_REPO_ROOT/overlay_ws/src/ego_swarm_real_bringup/rviz/ego_single_real.rviz"

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$NAV_SETUP_FILE"
set -u


ODOM_BRIDGE_TYPE="${ODOM_BRIDGE_TYPE:-fastlio2}"
PLANNER_INPUT_MODE="${PLANNER_INPUT_MODE:-fastlio2_leveled}"
EGO_TARGET_SYSTEM="${TARGET_SYSTEM:-2}"

export RMW_IMPLEMENTATION

cd "$OVERLAY_WS"


set +u
source "$OVERLAY_WS/install/setup.bash"
set -u

echo "Starting single-drone EGO-Swarm real-hardware bringup..."
echo "  drone_id=$DRONE_ID target_system=$EGO_TARGET_SYSTEM"
LAUNCH_ARGS=(
  odom_bridge_type:="$ODOM_BRIDGE_TYPE"
  planner_input_mode:="$PLANNER_INPUT_MODE"
  drone_id:="$DRONE_ID"
  target_system:="$EGO_TARGET_SYSTEM"
)
if [[ -n "$ROBOT_PARAMS_FILE" ]]; then
  if [[ ! -f "$ROBOT_PARAMS_FILE" ]]; then
    echo "Robot parameter file does not exist: $ROBOT_PARAMS_FILE" >&2
    exit 1
  fi
  LAUNCH_ARGS+=(robot_params_file:="$ROBOT_PARAMS_FILE")
fi
ros2 launch ego_swarm_real_bringup single_real.launch.py \
  "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!

sleep 3

echo "Opening RViz with single-drone real-hardware config..."
# rviz2 -d "$RVIZ_CONFIG_FILE"
wait "$LAUNCH_PID"
