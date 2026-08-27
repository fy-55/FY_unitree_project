#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
BINARY="${PROJECT_ROOT}/build/bin/g1_audio_client_test"
NETWORK_INTERFACE="${1:-}"
CONFIG_FILE="${G1_VOICE_LOCAL_CONFIG:-${SCRIPT_DIR}/g1_voice_local.env}"

if [[ -f "${CONFIG_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${CONFIG_FILE}"
  set +a
fi

export LLM_API_URL="${LLM_API_URL:-http://127.0.0.1:11434/v1/chat/completions}"
export LLM_API_KEY="${LLM_API_KEY:-ollama}"
export LLM_MODEL="${LLM_MODEL:-qwen3:8b}"
export LLM_REASONING_SPLIT="${LLM_REASONING_SPLIT:-0}"
export G1_WAKE_WORD="${G1_WAKE_WORD:-小贝同学}"
export G1_VOLUME="${G1_VOLUME:-60}"
export G1_ENABLE_WALKING="${G1_ENABLE_WALKING:-0}"
export G1_WALK_SPEED="${G1_WALK_SPEED:-0.12}"
export G1_WALK_DURATION_MS="${G1_WALK_DURATION_MS:-1500}"

if [[ ! -x "${BINARY}" ]]; then
  echo "错误：没有找到已编译程序 ${BINARY}"
  echo "请先编译目标g1_audio_client_test。"
  exit 1
fi

if [[ -z "${NETWORK_INTERFACE}" ]]; then
  echo "用法：$0 <G1有线网卡名>"
  exit 1
fi

if [[ ! -e "/sys/class/net/${NETWORK_INTERFACE}" ]]; then
  echo "错误：没有找到网卡 ${NETWORK_INTERFACE}。"
  ip -br link
  exit 1
fi

if ! curl -fsS "http://127.0.0.1:11434/api/version" >/dev/null 2>&1; then
  echo "错误：Ollama没有启动。请先运行 scripts/start_ollama.sh"
  exit 1
fi

echo "正在启动G1本地模型语音助手，模型：${LLM_MODEL}"
echo "使用网卡：${NETWORK_INTERFACE}，按Ctrl+C退出。"
exec "${BINARY}" "${NETWORK_INTERFACE}"
