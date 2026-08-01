#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OVERLAY_WS="$BASE_REPO_ROOT/overlay_ws"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DRONE_ID="${DRONE_ID:-1}"
TOPIC_PREFIX="/drone_${DRONE_ID}"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <bag_path>"
  exit 1
fi

BAG_PATH="$1"
if [[ ! -d "$BAG_PATH" ]]; then
  echo "Bag path does not exist: $BAG_PATH"
  exit 1
fi

set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$OVERLAY_WS/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

BAG_RATE="${BAG_RATE:-1.0}"
BAG_LOOP="${BAG_LOOP:-false}"
RVIZ_CONFIG_FILE="${RVIZ_CONFIG_FILE:-$BASE_REPO_ROOT/scripts/gvf_ismc_bag_replay.rviz}"

BAG_INFO="$(ros2 bag info "$BAG_PATH")"
RAW_ODOM_COUNT="$(
  printf '%s\n' "$BAG_INFO" |
    sed -n "s|.*Topic: ${TOPIC_PREFIX}/aft_mapped_to_init | Type: .* | Count: \\([0-9]\\+\\) |.*|\\1|p" |
    head -n 1
)"
LEVEL_ODOM_COUNT="$(
  printf '%s\n' "$BAG_INFO" |
    sed -n "s|.*Topic: ${TOPIC_PREFIX}/aft_mapped_to_init_level | Type: .* | Count: \\([0-9]\\+\\) |.*|\\1|p" |
    head -n 1
)"

RAW_ODOM_COUNT="${RAW_ODOM_COUNT:-0}"
LEVEL_ODOM_COUNT="${LEVEL_ODOM_COUNT:-0}"

if [[ "$RAW_ODOM_COUNT" == "0" && "$LEVEL_ODOM_COUNT" == "0" ]]; then
  echo "This bag does not contain recorded FAST-LIO odometry."
  echo "  ${TOPIC_PREFIX}/aft_mapped_to_init count: $RAW_ODOM_COUNT"
  echo "  ${TOPIC_PREFIX}/aft_mapped_to_init_level count: $LEVEL_ODOM_COUNT"
  echo "It cannot show the planned replay visualization."
  echo "Use 'ros2 bag info $BAG_PATH' to confirm the bag contents, then re-record with a healthy control chain."
  exit 1
fi

LOOP_ARGS=()
if [[ "$BAG_LOOP" == "true" ]]; then
  LOOP_ARGS+=(--loop)
fi

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

echo "Starting replay visualization..."
echo "  bag: $BAG_PATH"
echo "  drone_id: $DRONE_ID"
echo "  rate: $BAG_RATE"
echo "  loop: $BAG_LOOP"
echo "  rviz: $RVIZ_CONFIG_FILE"

ros2 launch gvf_ismc_real_bringup replay_visualization.launch.py \
  use_sim_time:=true \
  drone_id:="$DRONE_ID" \
  rviz_config_file:="$RVIZ_CONFIG_FILE" &
LAUNCH_PID=$!

sleep 2

echo "Playing bag..."
ros2 bag play "$BAG_PATH" \
  --clock \
  --rate "$BAG_RATE" \
  "${LOOP_ARGS[@]}"
