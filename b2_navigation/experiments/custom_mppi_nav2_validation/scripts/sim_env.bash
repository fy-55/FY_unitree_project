#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VALIDATION_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${VALIDATION_ROOT}/../.." && pwd)"
NAV_WS_INSTALL="${WORKSPACE_ROOT}/install"

source /opt/ros/humble/setup.bash
source "${NAV_WS_INSTALL}/setup.bash"

# This experiment is isolated from both the real B2 domain and the older
# official-only simulation.
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE

export GAZEBO_MASTER_URI=http://127.0.0.1:11346
export GAZEBO_IP=127.0.0.1
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models"

export ROS_HOME="${VALIDATION_ROOT}/.ros"
export ROS_LOG_DIR="${VALIDATION_ROOT}/log"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"

mkdir -p \
  "${ROS_HOME}" \
  "${ROS_LOG_DIR}" \
  "${VALIDATION_ROOT}/config/generated" \
  "${VALIDATION_ROOT}/results/runtime" \
  "${VALIDATION_ROOT}/results/trials"

export CUSTOM_MPPI_VALIDATION_ROOT="${VALIDATION_ROOT}"
