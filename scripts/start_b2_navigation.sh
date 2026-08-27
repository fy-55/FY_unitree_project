#!/usr/bin/env bash
set -euo pipefail

# Daily B2 navigation starter.
# Default behavior: open one terminal per node and do not save logs.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
SLAM_PARAMS="$WS_DIR/src/b2_navigation/params/slam_toolbox_localization.yaml"
NAV_PARAMS="$WS_DIR/src/b2_navigation/params/dog_nav_params.yaml"
MAP_BASE="${B2_MAP_BASE:-}"

START_WALK=false
START_RVIZ=false
SAVE_LOGS=false
LOG_DIR="$WS_DIR/logs/nav_start_$(date +%Y%m%d_%H%M%S)"

for arg in "$@"; do
  case "$arg" in
    --walk) START_WALK=true ;;
    --no-walk) START_WALK=false ;;
    --rviz) START_RVIZ=true ;;
    --log) SAVE_LOGS=true ;;
    --map-base=*) MAP_BASE="${arg#*=}" ;;
    -h|--help)
      echo "Usage: $0 [--map-base=/path/to/map] [--rviz] [--walk] [--log]"
      echo "  --map-base serialized SLAM Toolbox map path without .posegraph"
      echo "  --rviz     also start RViz"
      echo "  --walk     explicitly enable the physical B2 motion bridge"
      echo "  --no-walk  deprecated alias for the safe default"
      echo "  --log      save each terminal output under $WS_DIR/logs/"
      exit 0
      ;;
    *)
      echo "Unknown option: $arg"
      exit 1
      ;;
  esac
done

if [ "$START_WALK" = true ] && [ -z "$MAP_BASE" ]; then
  echo "Refusing --walk without --map-base. Run the no-motion preflight first."
  exit 2
fi

if [ -n "$MAP_BASE" ]; then
  if [[ ! "$MAP_BASE" =~ ^/[A-Za-z0-9._/-]+$ ]]; then
    echo "--map-base must be an absolute path using letters, digits, '.', '_', '-', and '/'."
    exit 2
  fi
  for extension in posegraph data; do
    if [ ! -f "${MAP_BASE}.${extension}" ]; then
      echo "Serialized map not found: ${MAP_BASE}.${extension}"
      exit 2
    fi
  done
fi

run_in_terminal() {
  local title="$1"
  local delay="$2"
  local cmd="$3"

  local setup="cd $WS_DIR; source $ROS_SETUP; source install/setup.bash; sleep $delay"
  local full_cmd="$setup; echo '===== $title ====='; $cmd"

  if [ "$SAVE_LOGS" = true ]; then
    mkdir -p "$LOG_DIR"
    full_cmd="$full_cmd 2>&1 | tee -a '$LOG_DIR/$title.log'"
  fi

  gnome-terminal --title="$title" -- bash -lc "$full_cmd; echo; read -p 'Press Enter to close...'"
}

run_in_terminal "01_b2_sensor" 0 \
  "ros2 launch b2_driver b2_sensor_launch.py"

run_in_terminal "02_pcl_to_scan" 2 \
  "ros2 launch b2_navigation pcl_to_scan.launch.py"

if [ -n "$MAP_BASE" ]; then
  run_in_terminal "03_slam_localization" 4 \
    "ros2 launch b2_navigation b2_localization.launch.py params_file:=$SLAM_PARAMS map_file_name:='$MAP_BASE' use_sim_time:=false"

  run_in_terminal "04_nav2" 6 \
    "ros2 launch b2_navigation b2_nav2_launch.py use_sim_time:=false autostart:=true params_file:=$NAV_PARAMS"
else
  echo "Preflight mode: sensors and scan conversion only."
  echo "Provide --map-base=/absolute/path/to/map to start localization and Nav2."
fi

if [ "$START_WALK" = true ]; then
  run_in_terminal "05_b2_walk" 8 \
    "ros2 run b2_driver b2_walk --ros-args -p cmd_vel_topic:=/cmd_vel -p enable_motion:=true"
else
  echo "Safe default: b2_walk was not started. Inspect with: ros2 topic echo /cmd_vel"
fi

if [ "$START_RVIZ" = true ]; then
  run_in_terminal "06_rviz" 10 \
    "ros2 launch nav2_bringup rviz_launch.py use_sim_time:=false"
fi

echo "Started requested B2 processes."
if [ "$SAVE_LOGS" = true ]; then
  echo "Logs: $LOG_DIR"
fi
