#!/usr/bin/env bash

set -eo pipefail

readonly DEFAULT_MAP_DIR="${G1_NAV_MAP_DIR:-${XDG_DATA_HOME:-${HOME}/.local/share}/unitree_g1_nav/maps}"
readonly SERIALIZE_SERVICE="/slam_toolbox/serialize_map"
readonly MAP_TOPIC="/map"

print_usage() {
  cat <<'EOF'
用法：
  ros2 run g1_nav_slam save_map.sh [地图名] [--force]
  ./save_map.sh [地图名] [--force]

示例：
  ros2 run g1_nav_slam save_map.sh lab_01

说明：
  1. 必须先启动真机基础链路和 slam_toolbox 建图节点。
  2. 保存前让机器人停稳，并等待约 3 秒。
  3. 地图名只能包含字母、数字、下划线和短横线。
  4. 不指定地图名时，自动使用 g1_map_年月日_时分秒。
  5. 默认保存到用户数据目录；可用 G1_NAV_MAP_DIR 指定其他目录。
  6. 默认拒绝覆盖同名文件；确认要覆盖时才添加 --force。

成功后会生成：
  地图名.pgm
  地图名.yaml
  地图名.posegraph
  地图名.data
EOF
}

map_name=""
force=false

for argument in "$@"; do
  case "$argument" in
    -h|--help)
      print_usage
      exit 0
      ;;
    --force)
      force=true
      ;;
    -* )
      echo "错误：未知选项 $argument" >&2
      print_usage >&2
      exit 2
      ;;
    *)
      if [[ -n "$map_name" ]]; then
        echo "错误：只能指定一个地图名。" >&2
        print_usage >&2
        exit 2
      fi
      map_name="$argument"
      ;;
  esac
done

if [[ -z "$map_name" ]]; then
  map_name="g1_map_$(date +%Y%m%d_%H%M%S)"
fi

if [[ ! "$map_name" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "错误：地图名只能包含字母、数字、下划线和短横线。" >&2
  exit 2
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [[ -n "${G1_NAV_WORKSPACE:-}" &&
      -f "${G1_NAV_WORKSPACE}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${G1_NAV_WORKSPACE}/install/setup.bash"
fi

# ROS 2 的 setup.bash 会读取若干尚未定义的内部变量，因此完成环境加载后
# 再启用 nounset，既避免误报，也保留后续变量拼写检查。
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "错误：找不到 ros2。请先安装并配置 ROS 2 Humble。" >&2
  exit 1
fi

readonly map_dir="$DEFAULT_MAP_DIR"
readonly map_base="${map_dir}/${map_name}"
readonly expected_files=(
  "${map_base}.pgm"
  "${map_base}.yaml"
  "${map_base}.posegraph"
  "${map_base}.data"
)

mkdir -p "$map_dir"

existing_files=()
for file in "${expected_files[@]}"; do
  if [[ -e "$file" ]]; then
    existing_files+=("$file")
  fi
done

if (( ${#existing_files[@]} > 0 )) && [[ "$force" != true ]]; then
  echo "错误：以下同名地图文件已经存在，为避免覆盖，本次未保存：" >&2
  printf '  %s\n' "${existing_files[@]}" >&2
  echo "请更换地图名；确认需要覆盖时添加 --force。" >&2
  exit 1
fi

service_type="$(timeout 5s ros2 service type "$SERIALIZE_SERVICE" 2>/dev/null || true)"
if [[ "$service_type" != "slam_toolbox/srv/SerializePoseGraph" ]]; then
  echo "错误：没有发现正在运行的 slam_toolbox 保存服务：$SERIALIZE_SERVICE" >&2
  echo "请先启动：ros2 launch g1_nav_slam mapping.launch.py use_sim_time:=false" >&2
  exit 1
fi

echo "正在等待 $MAP_TOPIC 地图数据……"
if ! timeout 10s ros2 topic echo "$MAP_TOPIC" --once --no-arr >/dev/null 2>&1; then
  echo "错误：10 秒内没有收到 $MAP_TOPIC。请确认 SLAM 已经开始建图。" >&2
  exit 1
fi

echo "保存目录：$map_dir"
echo "地图名称：$map_name"
echo "[1/2] 正在保存 slam_toolbox 位姿图……"

serialize_output="$(
  ros2 service call \
    "$SERIALIZE_SERVICE" \
    slam_toolbox/srv/SerializePoseGraph \
    "{filename: '${map_base}'}"
)"
printf '%s\n' "$serialize_output"

if ! grep -Eq 'result[=:][[:space:]]*0' <<<"$serialize_output"; then
  echo "错误：slam_toolbox 位姿图保存失败。" >&2
  exit 1
fi

echo "[2/2] 正在保存二维栅格地图……"
ros2 run nav2_map_server map_saver_cli \
  -t "$MAP_TOPIC" \
  -f "$map_base"

missing_files=()
for file in "${expected_files[@]}"; do
  if [[ ! -s "$file" ]]; then
    missing_files+=("$file")
  fi
done

if (( ${#missing_files[@]} > 0 )); then
  echo "错误：保存结束，但以下文件缺失或为空：" >&2
  printf '  %s\n' "${missing_files[@]}" >&2
  exit 1
fi

echo
echo "地图保存完成，四个文件均已生成："
ls -lh "${expected_files[@]}"
echo
echo "现在可以依次关闭 slam_toolbox、RViz 和真机基础链路。"
