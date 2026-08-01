#!/usr/bin/env bash

# Source this file from launch helpers. It keeps robot-specific values out of
# tracked launch files while allowing explicit command-line environment values
# to take precedence for one-off diagnostics.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this helper from a launch script; do not execute it directly." >&2
  exit 2
fi

ROBOT_CONFIG_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROBOT_CONFIG_REPO_ROOT

if [[ -z "${ROBOT_CONFIG:-}" ]]; then
  ROBOT_CONFIG="$ROBOT_CONFIG_REPO_ROOT/config/robot.env"
fi

robot_config_names=(
  DRONE_ID
  TARGET_SYSTEM
  PX4_DDS_NAMESPACE
  ROS_DOMAIN_ID
  ROS_DISTRO
  RMW_IMPLEMENTATION
  LIVOX_CONFIG_FILE
  ROBOT_PARAMS_FILE
  PX4_DIR
)
robot_config_external_names=()

for robot_config_name in "${robot_config_names[@]}"; do
  if [[ -v "$robot_config_name" ]]; then
    printf -v "robot_config_external_${robot_config_name}" '%s' "${!robot_config_name}"
    robot_config_external_names+=("$robot_config_name")
  fi
done

if [[ -f "$ROBOT_CONFIG" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$ROBOT_CONFIG"
  set +a
fi

for robot_config_name in "${robot_config_external_names[@]}"; do
  robot_config_saved_name="robot_config_external_${robot_config_name}"
  printf -v "$robot_config_name" '%s' "${!robot_config_saved_name}"
  unset "$robot_config_saved_name"
done

: "${DRONE_ID:=1}"
: "${TARGET_SYSTEM:=}"
: "${PX4_DDS_NAMESPACE:=drone_${DRONE_ID}}"
: "${ROS_DOMAIN_ID:=0}"
: "${ROS_DISTRO:=jazzy}"
: "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"
: "${LIVOX_CONFIG_FILE:=}"
: "${ROBOT_PARAMS_FILE:=}"

export ROBOT_CONFIG
export DRONE_ID TARGET_SYSTEM PX4_DDS_NAMESPACE ROS_DOMAIN_ID ROS_DISTRO
export RMW_IMPLEMENTATION LIVOX_CONFIG_FILE ROBOT_PARAMS_FILE

unset robot_config_name robot_config_names robot_config_external_names
unset robot_config_saved_name
