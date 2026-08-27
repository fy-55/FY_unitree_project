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

OLLAMA_ROOT="${OLLAMA_ROOT:-}"
if [[ -n "${OLLAMA_ROOT}" ]]; then
  OLLAMA_BIN="${OLLAMA_ROOT}/bin/ollama"
else
  OLLAMA_BIN="$(command -v ollama || true)"
fi
if [[ -n "${OLLAMA_MODELS:-}" ]]; then
  export OLLAMA_MODELS
fi
export OLLAMA_HOST="127.0.0.1:11434"
export OLLAMA_CONTEXT_LENGTH="2048"
export OLLAMA_FLASH_ATTENTION="1"
export OLLAMA_KEEP_ALIVE="30m"

if [[ -z "${OLLAMA_BIN}" || ! -x "${OLLAMA_BIN}" ]]; then
  echo "错误：没有找到 Ollama。请安装 Ollama，或设置 OLLAMA_ROOT。"
  exit 1
fi

if curl -fsS "http://${OLLAMA_HOST}/api/version" >/dev/null 2>&1; then
  echo "Ollama 已经在运行，不需要重复启动。"
  exit 0
fi

echo "正在启动 Ollama。请保持这个终端窗口开启。"
exec "${OLLAMA_BIN}" serve
