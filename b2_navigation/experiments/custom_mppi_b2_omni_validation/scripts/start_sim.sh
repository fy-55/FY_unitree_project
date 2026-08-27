#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/sim_env.bash"
set -euo pipefail

variant="${1:-custom}"
case "${variant}" in
  custom)
    params_file="${B2_OMNI_VALIDATION_ROOT}/config/generated/custom_gpu_omni.yaml"
    ;;
  official)
    params_file="${B2_OMNI_VALIDATION_ROOT}/config/generated/official_omni.yaml"
    ;;
  *)
    printf 'Usage: %s [custom|official]\n' "$0" >&2
    exit 2
    ;;
esac

python3 "${B2_OMNI_VALIDATION_ROOT}/prepare_configs.py"
bash "${SCRIPT_DIR}/check_isolation.sh"

printf '\nStarting isolated B2-style omnidirectional simulation\n'
printf 'variant=%s\nparams=%s\n' "${variant}" "${params_file}"
printf 'domain=%s, gazebo=%s\n\n' \
  "${ROS_DOMAIN_ID}" "${GAZEBO_MASTER_URI}"

exec ros2 launch \
  "${B2_OMNI_VALIDATION_ROOT}/launch/b2_omni_validation.launch.py" \
  params_file:="${params_file}"
