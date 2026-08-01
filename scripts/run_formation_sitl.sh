#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/load_robot_config.sh"

FORMATION_WS="${FORMATION_WS:-$REPO_ROOT/uav_formation_ws}"
PX4_DIR="${PX4_DIR:-$HOME/PX4/PX4-Autopilot}"
TOTAL_UAVS="${TOTAL_UAVS:-3}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash}"
source "$ROS_SETUP"
source "$FORMATION_WS/install/setup.bash"
exec ros2 launch distribute_control multi_uav_launch.py \
  total_uavs:="$TOTAL_UAVS" \
  px4_dir:="$PX4_DIR" \
  start_px4:=true \
  start_rviz:="${START_RVIZ:-false}"
