#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/sim_env.bash"
set -euo pipefail

"${SCRIPT_DIR}/check_isolation.sh"

normalize_launch_boolean() {
  case "${1,,}" in
    true|1|yes|on)
      printf 'True'
      ;;
    false|0|no|off)
      printf 'False'
      ;;
    *)
      printf 'Invalid boolean value: %s\n' "$1" >&2
      return 1
      ;;
  esac
}

# Humble's tb3_simulation_launch.py inserts these values into a PythonExpression,
# so it requires Python spelling (True/False), not lowercase shell spelling.
# The current default shows both Gazebo and RViz. Set NAV2_SIM_HEADLESS=true
# to hide only gzclient if running two OpenGL windows makes RViz unstable.
NAV2_SIM_HEADLESS="$(normalize_launch_boolean "${NAV2_SIM_HEADLESS:-false}")"
NAV2_SIM_USE_RVIZ="$(normalize_launch_boolean "${NAV2_SIM_USE_RVIZ:-true}")"
# Separate Nav2 processes are easier to inspect and shut down cleanly on Humble.
NAV2_SIM_USE_COMPOSITION="$(
  normalize_launch_boolean "${NAV2_SIM_USE_COMPOSITION:-false}"
)"
PARAMS_FILE="${NAV2_SIM_ROOT}/config/nav2_official_mppi_params.yaml"

existing_nodes="$(timeout 5s ros2 node list 2>/dev/null || true)"
if grep -Eq '^/(amcl|bt_navigator|controller_server|map_server)$' \
    <<< "${existing_nodes}"; then
  printf '\n[ERROR] Another Nav2 simulation is already running in ROS_DOMAIN_ID=%s.\n' \
    "${ROS_DOMAIN_ID}" >&2
  printf 'Stop its start_sim.sh terminal with Ctrl+C before starting a new one.\n' >&2
  printf 'Existing core nodes:\n%s\n' \
    "$(grep -E '^/(amcl|bt_navigator|controller_server|map_server)$' \
      <<< "${existing_nodes}")" >&2
  exit 1
fi

printf '\nStarting official Nav2 TurtleBot3 + MPPI simulation...\n'
printf 'params_file=%s\n' "${PARAMS_FILE}"
printf 'headless=%s, use_rviz=%s, use_composition=%s\n' \
  "${NAV2_SIM_HEADLESS}" "${NAV2_SIM_USE_RVIZ}" \
  "${NAV2_SIM_USE_COMPOSITION}"
printf 'headless=True hides only the Gazebo window; Gazebo physics still runs.\n'
printf 'This script does not source nav_ws/install and does not start any B2 node.\n\n'

exec ros2 launch nav2_bringup tb3_simulation_launch.py \
  params_file:="${PARAMS_FILE}" \
  headless:="${NAV2_SIM_HEADLESS}" \
  use_rviz:="${NAV2_SIM_USE_RVIZ}" \
  use_composition:="${NAV2_SIM_USE_COMPOSITION}" \
  use_sim_time:=true
