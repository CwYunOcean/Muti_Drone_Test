#!/usr/bin/env bash
set -eo pipefail

# Record the EGO-Swarm multi-drone experiment topic set.
# DRONE_IDS: space separated list of drone ids present in the experiment.
#   Single drone:  DRONE_IDS="1" ./record_ego_swarm_experiment.sh
#   Two drones:    DRONE_IDS="0 1" ./record_ego_swarm_experiment.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

DRONE_IDS="${DRONE_IDS:-1}"
BAG_DIR="${BAG_DIR:-$BASE_REPO_ROOT/bags}"
BAG_NAME="${BAG_NAME:-ego_swarm_$(date +%Y%m%d_%H%M%S)}"
BAG_PATH="$BAG_DIR/$BAG_NAME"

set +u
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

mkdir -p "$BAG_DIR"

TOPICS=(
  /broadcast_bspline
  /tf
  /tf_static
)

for id in $DRONE_IDS; do
  TOPICS+=(
    "/drone_${id}/aft_mapped_to_init"
    "/drone_${id}/aft_mapped_to_init_level"
    "/drone_${id}/cloud_registered_level"
    "/drone_${id}_planning/pos_cmd"
    "/drone_${id}_planning/bspline"
    "/drone_${id}_planning/swarm_trajs"
    "/drone_${id}_grid/grid_map/occupancy_inflate"
    "/drone_${id}/fmu/out/vehicle_odometry"
    "/drone_${id}/fmu/in/trajectory_setpoint"
  )
done

echo "Recording to $BAG_PATH"
printf '  %s\n' "${TOPICS[@]}"

ros2 bag record -o "$BAG_PATH" "${TOPICS[@]}"
