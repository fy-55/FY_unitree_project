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
export OLLAMA_HOST="127.0.0.1:11434"
export OLLAMA_CONTEXT_LENGTH="2048"
export OLLAMA_FLASH_ATTENTION="1"
export OLLAMA_KEEP_ALIVE="30m"

if [[ ! -x "${OLLAMA_ROOT}/bin/ollama" ]]; then
  echo "错误：没有找到 ${OLLAMA_ROOT}/bin/ollama"
  exit 1
fi

if curl -fsS "http://${OLLAMA_HOST}/api/version" >/dev/null 2>&1; then
  echo "Ollama 已经在运行，不需要重复启动。"
  exit 0
fi

echo "正在启动 Ollama。请保持这个终端窗口开启。"
exec "${OLLAMA_ROOT}/bin/ollama" serve
