#!/usr/bin/env bash

# Source this file in every terminal participating in the local G1 simulation.
# It is intentionally separate from use_g1_ros2, which binds CycloneDDS to the
# physical Unitree Ethernet interface.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=80
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$WORKSPACE_DIR/src/g1_nav_sim/config/cyclonedds_sim.xml"

echo "G1 simulation ROS 2 environment ready: CycloneDDS, domain 80, localhost, participant indexes 0..50"
