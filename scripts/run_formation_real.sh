#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/load_robot_config.sh"

FORMATION_WS="${FORMATION_WS:-$REPO_ROOT/uav_formation_ws}"
FORMATION_TARGET_SYSTEM="${TARGET_SYSTEM:-$DRONE_ID}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash}"
source "$ROS_SETUP"
source "$FORMATION_WS/install/setup.bash"
exec ros2 launch distribute_control single_real.launch.py \
  drone_id:="$DRONE_ID" \
  target_system:="$FORMATION_TARGET_SYSTEM"
