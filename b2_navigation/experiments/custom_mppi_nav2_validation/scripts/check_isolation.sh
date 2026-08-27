#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/sim_env.bash"
set -euo pipefail

failed=0

check_prefix() {
  local package_name="$1"
  local expected_prefix="$2"
  local actual_prefix=""
  if ! actual_prefix="$(ros2 pkg prefix "${package_name}" 2>/dev/null)"; then
    printf '[FAIL] package missing: %s\n' "${package_name}"
    failed=1
    return
  fi
  if [[ "${actual_prefix}" != "${expected_prefix}"* ]]; then
    printf '[FAIL] unexpected package: %s -> %s\n' \
      "${package_name}" "${actual_prefix}"
    failed=1
    return
  fi
  printf '[OK] package: %s -> %s\n' "${package_name}" "${actual_prefix}"
}

printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID}"
printf 'ROS_LOCALHOST_ONLY=%s\n' "${ROS_LOCALHOST_ONLY}"
printf 'GAZEBO_MASTER_URI=%s\n' "${GAZEBO_MASTER_URI}"
printf 'ROS_LOG_DIR=%s\n\n' "${ROS_LOG_DIR}"

[[ "${ROS_DOMAIN_ID}" == "43" ]] || failed=1
[[ "${ROS_LOCALHOST_ONLY}" == "1" ]] || failed=1
[[ "${GAZEBO_MASTER_URI}" == "http://127.0.0.1:11346" ]] || failed=1

check_prefix nav2_bringup "/opt/ros/humble"
check_prefix nav2_mppi_controller "/opt/ros/humble"
check_prefix gazebo_ros "/opt/ros/humble"
check_prefix turtlebot3_gazebo "/opt/ros/humble"
check_prefix nav2_custom_plugins "${NAV_WS_INSTALL}"

PLUGIN_XML="${NAV_WS_INSTALL}/nav2_custom_plugins/share/nav2_custom_plugins/plugins.xml"
PLUGIN_LIBRARY="${NAV_WS_INSTALL}/nav2_custom_plugins/lib/libnav2_custom_plugins_gpu.so"
if rg -q 'nav2_custom_plugins/MPPIGPUController' "${PLUGIN_XML}" &&
   [[ -r "${PLUGIN_LIBRARY}" ]]; then
  printf '[OK] custom GPU-MPPI plugin metadata and library\n'
else
  printf '[FAIL] custom GPU-MPPI plugin metadata/library\n'
  failed=1
fi

existing_nodes="$(timeout 5s ros2 node list 2>/dev/null || true)"
if grep -Eq '^/(amcl|bt_navigator|controller_server|map_server)$' \
    <<< "${existing_nodes}"; then
  printf '[FAIL] another Nav2 stack is already running in domain 43\n'
  failed=1
fi

if (( failed != 0 )); then
  printf '\n[NOT READY] Isolation or dependency check failed.\n'
  exit 2
fi

printf '\n[READY] Isolated custom-MPPI validation environment is ready.\n'
