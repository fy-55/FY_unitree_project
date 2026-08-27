#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/sim_env.bash"
set -euo pipefail

RUN_ID="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="${NAV2_SIM_ROOT}/data/mppi_${RUN_ID}"

printf 'Recording the isolated simulation to:\n%s\n' "${BAG_DIR}"
printf 'Stop recording with Ctrl+C after the robot reaches the goal.\n\n'

exec ros2 bag record \
  --output "${BAG_DIR}" \
  /cmd_vel_nav \
  /cmd_vel \
  /odom \
  /plan \
  /scan \
  /tf \
  /tf_static \
  /local_costmap/costmap \
  /trajectories \
  /transformed_global_plan
