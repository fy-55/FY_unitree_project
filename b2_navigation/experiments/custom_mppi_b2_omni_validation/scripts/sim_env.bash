#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export B2_OMNI_VALIDATION_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
export B2_OMNI_WORKSPACE_ROOT="$(cd "${B2_OMNI_VALIDATION_ROOT}/../.." && pwd)"

source /opt/ros/humble/setup.bash
source "${B2_OMNI_WORKSPACE_ROOT}/install/setup.bash"
export ROS_DOMAIN_ID=44
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11347
export ROS_LOG_DIR="${B2_OMNI_VALIDATION_ROOT}/log"
mkdir -p "${ROS_LOG_DIR}"
