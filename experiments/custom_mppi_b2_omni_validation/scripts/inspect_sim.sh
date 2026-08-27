#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/sim_env.bash"
set -euo pipefail

printf '%s\n' '[topics]'
ros2 topic list
printf '%s\n' '[nodes]'
ros2 node list
printf '%s\n' '[Gazebo models]'
timeout 5 gz model -l || true
printf '%s\n' '[Gazebo transport topics]'
timeout 5 gz topic -l || true
printf '%s\n' '[odom one sample]'
timeout 3 ros2 topic echo /odom --once || true
printf '%s\n' '[scan one sample]'
timeout 3 ros2 topic echo /scan --once || true
printf '%s\n' '[clock one sample]'
timeout 3 ros2 topic echo /clock --once || true
