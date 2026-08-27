#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/sim_env.bash"
set -euo pipefail

check_prefix() {
  local package="$1"
  local expected="$2"
  local actual
  actual="$(ros2 pkg prefix "${package}")"
  if [[ "${actual}" != "${expected}"* ]]; then
    printf '[ERROR] %s resolved to %s, expected prefix %s\n' \
      "${package}" "${actual}" "${expected}" >&2
    return 1
  fi
  printf '[OK] package: %s -> %s\n' "${package}" "${actual}"
}

printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID}"
printf 'ROS_LOCALHOST_ONLY=%s\n' "${ROS_LOCALHOST_ONLY}"
printf 'GAZEBO_MASTER_URI=%s\n' "${GAZEBO_MASTER_URI}"
printf 'ROS_LOG_DIR=%s\n\n' "${ROS_LOG_DIR}"

check_prefix nav2_bringup /opt/ros/humble
check_prefix nav2_mppi_controller /opt/ros/humble
check_prefix gazebo_ros /opt/ros/humble
check_prefix gazebo_plugins /opt/ros/humble
check_prefix nav2_custom_plugins "${B2_OMNI_WORKSPACE_ROOT}/install/nav2_custom_plugins"

test -f /opt/ros/humble/lib/libgazebo_ros_planar_move.so
test -f /opt/ros/humble/lib/libgazebo_ros_ray_sensor.so
test -f "${B2_OMNI_VALIDATION_ROOT}/worlds/b2_omni_world.world"
test -f "${B2_OMNI_VALIDATION_ROOT}/launch/b2_omni_validation.launch.py"

printf '\n[READY] Isolated B2-style omnidirectional environment is ready.\n'
