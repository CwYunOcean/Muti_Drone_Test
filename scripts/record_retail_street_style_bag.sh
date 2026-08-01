#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

DEPENDENCY_REPO_ROOT="$BASE_REPO_ROOT"
SLAM_SETUP_FILE="$BASE_REPO_ROOT/slam_ws/install/setup.bash"
OVERLAY_SETUP_FILE="$BASE_REPO_ROOT/overlay_ws/install/setup.bash"

if ([[ ! -f "$SLAM_SETUP_FILE" ]] || [[ ! -f "$OVERLAY_SETUP_FILE" ]]) && [[ "$BASE_REPO_ROOT" == */.worktrees/* ]]; then
  DEPENDENCY_REPO_ROOT="${BASE_REPO_ROOT%%/.worktrees/*}"
  SLAM_SETUP_FILE="$DEPENDENCY_REPO_ROOT/slam_ws/install/setup.bash"
  OVERLAY_SETUP_FILE="$DEPENDENCY_REPO_ROOT/overlay_ws/install/setup.bash"
fi

BAG_BASE_DIR="${RETAIL_STREET_BAG_OUTPUT_DIR:-/tmp/retail_street_style_bags}"
BAG_NAME="${RETAIL_STREET_BAG_NAME:-$(date +%F_%H-%M-%S)_retail_street_style}"
BAG_PATH="$BAG_BASE_DIR/$BAG_NAME"

cleanup() {
  if [[ -n "${REPUBLISH_PID:-}" ]] && kill -0 "$REPUBLISH_PID" 2>/dev/null; then
    kill "$REPUBLISH_PID" 2>/dev/null || true
    wait "$REPUBLISH_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

set +u
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$SLAM_SETUP_FILE"
if [[ -f "$OVERLAY_SETUP_FILE" ]]; then
  source "$OVERLAY_SETUP_FILE"
fi
set -u

mkdir -p "$BAG_BASE_DIR"

echo "Republishing /camera/camera/infra1/image_rect_raw as /left_camera/image ..."
ros2 run image_transport republish raw raw \
  --ros-args \
  --remap in:=/camera/camera/infra1/image_rect_raw \
  --remap out:=/left_camera/image &
REPUBLISH_PID=$!

sleep 1

echo "Recording Retail_Street-style bag to $BAG_PATH"
DRONE_ID="${DRONE_ID:-1}"
TOPIC_PREFIX="/drone_${DRONE_ID}"

echo "Topics:"
echo "  ${TOPIC_PREFIX}/livox/lidar"
echo "  ${TOPIC_PREFIX}/livox/imu"
echo "  /left_camera/image"
echo "  /tf"
echo "  /tf_static"

ros2 bag record -o "$BAG_PATH" \
  "${TOPIC_PREFIX}/livox/lidar" \
  "${TOPIC_PREFIX}/livox/imu" \
  /left_camera/image \
  /tf \
  /tf_static
