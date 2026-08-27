#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${G1_VOICE_LOCAL_CONFIG:-${SCRIPT_DIR}/g1_voice_local.env}"
if [[ -f "${CONFIG_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${CONFIG_FILE}"
  set +a
fi

OLLAMA_ROOT="${OLLAMA_ROOT:-/home/oem/fy_sim/ollama}"
export OLLAMA_MODELS="${OLLAMA_MODELS:-/home/oem/fy_sim/ollama_models}"
LLM_MODEL="${LLM_MODEL:-qwen3:8b}"

if ! curl -fsS "http://127.0.0.1:11434/api/version" >/dev/null 2>&1; then
  echo "错误：Ollama 没有启动。请先运行 scripts/start_ollama.sh"
  exit 1
fi

echo "已进入本地模型文字对话。输入 /bye 退出。"
exec "${OLLAMA_ROOT}/bin/ollama" run "${LLM_MODEL}" --think=false
