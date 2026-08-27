#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/sim_env.bash"
set -euo pipefail

missing=0

check_command() {
  local command_name="$1"
  if command -v "${command_name}" >/dev/null 2>&1; then
    printf '[OK] command: %s\n' "${command_name}"
  else
    printf '[MISSING] command: %s\n' "${command_name}"
    missing=1
  fi
}

check_ros_package() {
  local package_name="$1"
  local package_prefix=""
  if package_prefix="$(ros2 pkg prefix "${package_name}" 2>/dev/null)"; then
    if [[ "${package_name}" == "nav2_bringup" ||
          "${package_name}" == "nav2_mppi_controller" ]]; then
      if [[ "${package_prefix}" != /opt/ros/humble* ]]; then
        printf '[FAIL] ROS package is not from /opt/ros/humble: %s -> %s\n' \
          "${package_name}" "${package_prefix}"
        missing=1
        return
      fi
    fi
    printf '[OK] ROS package: %s -> %s\n' "${package_name}" "${package_prefix}"
  else
    printf '[MISSING] ROS package: %s\n' "${package_name}"
    missing=1
  fi
}

printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID}"
printf 'ROS_LOCALHOST_ONLY=%s\n' "${ROS_LOCALHOST_ONLY}"
printf 'GAZEBO_MASTER_URI=%s\n' "${GAZEBO_MASTER_URI}"
printf 'ROS_HOME=%s\n\n' "${ROS_HOME}"

check_command gzserver
check_command gzclient
check_ros_package nav2_bringup
check_ros_package nav2_mppi_controller
check_ros_package gazebo_ros
check_ros_package gazebo_plugins
check_ros_package turtlebot3_gazebo

if [[ -f /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_world/model.config ]]; then
  printf '[OK] Gazebo model: turtlebot3_world\n'
else
  printf '[MISSING] Gazebo model: turtlebot3_world\n'
  missing=1
fi

if [[ "${ROS_DOMAIN_ID}" != "42" || "${ROS_LOCALHOST_ONLY}" != "1" ]]; then
  printf '\n[FAIL] DDS isolation variables are not correct.\n'
  exit 1
fi

if [[ "${GAZEBO_MASTER_URI}" != "http://127.0.0.1:11345" ]]; then
  printf '\n[FAIL] Gazebo is not using the private loopback master.\n'
  exit 1
fi

if [[ -n "${GAZEBO_MODEL_DATABASE_URI}" ]]; then
  printf '\n[FAIL] Gazebo online model database has not been disabled.\n'
  exit 1
fi

for path_variable in \
  AMENT_PREFIX_PATH \
  CMAKE_PREFIX_PATH \
  COLCON_PREFIX_PATH \
  LD_LIBRARY_PATH \
  PYTHONPATH \
  PATH
do
  if [[ "${!path_variable-}" == *"${SIM_FORBIDDEN_OVERLAY}"* ]]; then
    printf '\n[FAIL] B2 overlay remains in %s.\n' "${path_variable}"
    exit 1
  fi
done

if (( missing != 0 )); then
  printf '\n[NOT READY] Simulation dependencies/models are missing. Run the install command in README.md.\n'
  exit 2
fi

printf '\n[READY] The official MPPI simulation environment is isolated and complete.\n'
