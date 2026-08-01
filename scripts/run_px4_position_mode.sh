#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$BASE_REPO_ROOT/scripts/load_robot_config.sh"

OVERLAY_WS="$BASE_REPO_ROOT/overlay_ws"

LEVELING_CONFIG="$OVERLAY_WS/src/fastlio2_to_ego_swarm_leveling/config/fastlio2_to_ego_swarm_leveling.yaml"
PX4_ODOM_CONFIG="$OVERLAY_WS/src/fastlio2_to_px4_odometry/config/fastlio2_to_px4_odometry.yaml"

PIDS=()

cleanup() {
  trap - EXIT INT TERM HUP

  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done

  for pid in "${PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}

trap cleanup EXIT INT TERM HUP

set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$OVERLAY_WS/install/setup.bash"
set -u

export RMW_IMPLEMENTATION

TOPIC_PREFIX="/drone_${DRONE_ID}"

echo "Starting FAST-LIO2 leveling for PX4 Position mode..."
echo "  ${TOPIC_PREFIX}/aft_mapped_to_init -> ${TOPIC_PREFIX}/aft_mapped_to_init_level"
ros2 run fastlio2_to_ego_swarm_leveling fastlio2_to_ego_swarm_leveling_node \
  --ros-args \
  --params-file "$LEVELING_CONFIG" \
  -p input_odom_topic:="${TOPIC_PREFIX}/aft_mapped_to_init" \
  -p input_cloud_topic:="${TOPIC_PREFIX}/cloud_registered" \
  -p output_odom_topic:="${TOPIC_PREFIX}/aft_mapped_to_init_level" \
  -p output_cloud_topic:="${TOPIC_PREFIX}/cloud_registered_level" &
PIDS+=("$!")

sleep 1

echo "Starting PX4 visual odometry bridge for Position mode..."
echo "  ${TOPIC_PREFIX}/aft_mapped_to_init_level -> ${TOPIC_PREFIX}/fmu/in/vehicle_visual_odometry"
ros2 run fastlio2_to_px4_odometry fastlio2_to_px4_odometry_node \
  --ros-args \
  --params-file "$PX4_ODOM_CONFIG" \
  -p input_topic:="${TOPIC_PREFIX}/aft_mapped_to_init_level" \
  -p output_topic:="${TOPIC_PREFIX}/fmu/in/vehicle_visual_odometry" &
PIDS+=("$!")

echo
echo "This script only publishes visual odometry to PX4."
echo "It does not publish Offboard setpoints or vehicle commands."
echo
echo "Check before flying:"
echo "  ros2 topic hz ${TOPIC_PREFIX}/aft_mapped_to_init_level"
echo "  ros2 topic hz ${TOPIC_PREFIX}/fmu/in/vehicle_visual_odometry"

wait -n "${PIDS[@]}"
