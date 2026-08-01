#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OVERLAY_WS="$BASE_REPO_ROOT/overlay_ws"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DRONE_ID="${DRONE_ID:-1}"
TOPIC_PREFIX="/drone_${DRONE_ID}"
PLANNING_PREFIX="/drone_${DRONE_ID}_planning"
FMU_IN_PREFIX="${TOPIC_PREFIX}/fmu/in"
FMU_OUT_PREFIX="${TOPIC_PREFIX}/fmu/out"

set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$OVERLAY_WS/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

BAG_DIR="${BAG_DIR:-$BASE_REPO_ROOT/bags}"
RUN_TAG="${RUN_TAG:-gvf_ismc_$(date +%Y%m%d_%H%M%S)}"
OUTPUT_PATH="$BAG_DIR/$RUN_TAG"

mkdir -p "$BAG_DIR"

echo "Recording GVF/ISMC experiment bag to: $OUTPUT_PATH"
echo "drone_id=$DRONE_ID"
echo "Topics:"
echo "  ${TOPIC_PREFIX}/aft_mapped_to_init"
echo "  ${TOPIC_PREFIX}/aft_mapped_to_init_level"
echo "  /gvf/reference"
echo "  /gvf/reference_path"
echo "  /gvf/reference_marker"
echo "  ${PLANNING_PREFIX}/pos_cmd"
echo "  /tf"
echo "  /tf_static"
echo "  ${FMU_IN_PREFIX}/offboard_control_mode"
echo "  ${FMU_IN_PREFIX}/trajectory_setpoint"
echo "  ${FMU_IN_PREFIX}/vehicle_command"
echo "  ${FMU_OUT_PREFIX}/vehicle_status_v1"
echo "  ${FMU_OUT_PREFIX}/vehicle_odometry"

cd "$BASE_REPO_ROOT"

echo "Checking required topics before recording..."
ros2 topic echo "${TOPIC_PREFIX}/aft_mapped_to_init" --once >/dev/null
ros2 topic echo "${TOPIC_PREFIX}/aft_mapped_to_init_level" --once >/dev/null
ros2 topic echo /gvf/reference --once >/dev/null
ros2 topic echo /gvf/reference_path --once >/dev/null
ros2 topic echo /gvf/reference_marker --once >/dev/null
ros2 topic echo "${PLANNING_PREFIX}/pos_cmd" --once >/dev/null
echo "Required topics are live. Starting rosbag record."

ros2 bag record \
  -o "$OUTPUT_PATH" \
  "${TOPIC_PREFIX}/aft_mapped_to_init" \
  "${TOPIC_PREFIX}/aft_mapped_to_init_level" \
  /gvf/reference \
  /gvf/reference_path \
  /gvf/reference_marker \
  "${PLANNING_PREFIX}/pos_cmd" \
  /tf \
  /tf_static \
  "${FMU_IN_PREFIX}/offboard_control_mode" \
  "${FMU_IN_PREFIX}/trajectory_setpoint" \
  "${FMU_IN_PREFIX}/vehicle_command" \
  "${FMU_OUT_PREFIX}/vehicle_status_v1" \
  "${FMU_OUT_PREFIX}/vehicle_odometry"
