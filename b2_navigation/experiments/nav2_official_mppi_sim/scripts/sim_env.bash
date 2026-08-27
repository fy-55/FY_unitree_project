#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIM_EXPERIMENT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SIM_WORKSPACE_ROOT="$(cd "${SIM_EXPERIMENT_ROOT}/../.." && pwd)"
SIM_FORBIDDEN_OVERLAY="${SIM_WORKSPACE_ROOT}/install"

# A terminal may already have sourced nav_ws/install before this script starts.
# Remove that inherited overlay from every common ROS/loader search path.
remove_overlay_from_path_variable() {
  local variable_name="$1"
  local original_value="${!variable_name-}"
  local cleaned_value=""
  local entry=""

  IFS=':' read -r -a entries <<< "${original_value}"
  for entry in "${entries[@]}"; do
    if [[ -z "${entry}" ||
          "${entry}" == "${SIM_FORBIDDEN_OVERLAY}" ||
          "${entry}" == "${SIM_FORBIDDEN_OVERLAY}/"* ]]; then
      continue
    fi
    if [[ -n "${cleaned_value}" ]]; then
      cleaned_value+=":"
    fi
    cleaned_value+="${entry}"
  done

  printf -v "${variable_name}" '%s' "${cleaned_value}"
  export "${variable_name}"
}

for path_variable in \
  AMENT_PREFIX_PATH \
  CMAKE_PREFIX_PATH \
  COLCON_PREFIX_PATH \
  LD_LIBRARY_PATH \
  PYTHONPATH \
  PATH
do
  remove_overlay_from_path_variable "${path_variable}"
done

# Load only the ROS Humble underlay after removing inherited B2 overlay paths.
source /opt/ros/humble/setup.bash

# DDS isolation: simulation nodes use domain 42 and communicate only through
# loopback. The real B2 stack normally uses another domain/network interface.
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE

# Gazebo Classic isolation: private master port and loopback-only transport.
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export GAZEBO_IP=127.0.0.1

# Use only the locally installed official TurtleBot3 and Gazebo models. The
# legacy online model database is slow/unavailable and must not be a hidden
# runtime dependency of a reproducible experiment.
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models"

# Keep ROS logs and CLI state out of the normal ~/.ros directory.
export ROS_HOME="${SIM_EXPERIMENT_ROOT}/.ros"
export ROS_LOG_DIR="${SIM_EXPERIMENT_ROOT}/log"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

export NAV2_SIM_ROOT="${SIM_EXPERIMENT_ROOT}"
