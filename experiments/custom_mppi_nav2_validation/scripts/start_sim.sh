#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/sim_env.bash"
set -euo pipefail

variant="${1:-custom}"
case "${variant}" in
  custom)
    params_file="${CUSTOM_MPPI_VALIDATION_ROOT}/config/generated/custom_gpu_tb3.yaml"
    ;;
  official)
    params_file="${CUSTOM_MPPI_VALIDATION_ROOT}/config/generated/official_reference.yaml"
    ;;
  *)
    printf 'Usage: %s [custom|official]\n' "$0" >&2
    exit 2
    ;;
esac

normalize_launch_boolean() {
  case "${1,,}" in
    true|1|yes|on) printf 'True' ;;
    false|0|no|off) printf 'False' ;;
    *)
      printf 'Invalid boolean: %s\n' "$1" >&2
      return 1
      ;;
  esac
}

python3 "${CUSTOM_MPPI_VALIDATION_ROOT}/prepare_configs.py"
bash "${SCRIPT_DIR}/check_isolation.sh"

headless="$(
  normalize_launch_boolean "${CUSTOM_MPPI_HEADLESS:-true}"
)"
use_rviz="$(
  normalize_launch_boolean "${CUSTOM_MPPI_USE_RVIZ:-false}"
)"

printf '\nStarting isolated TurtleBot3 validation\n'
printf 'variant=%s\nparams=%s\n' "${variant}" "${params_file}"
printf 'domain=%s, gazebo=%s, headless=%s, rviz=%s\n\n' \
  "${ROS_DOMAIN_ID}" "${GAZEBO_MASTER_URI}" "${headless}" "${use_rviz}"

exec ros2 launch nav2_bringup tb3_simulation_launch.py \
  params_file:="${params_file}" \
  headless:="${headless}" \
  use_rviz:="${use_rviz}" \
  use_composition:=False \
  use_sim_time:=True
