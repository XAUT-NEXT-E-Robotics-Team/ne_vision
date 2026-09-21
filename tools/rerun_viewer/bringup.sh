#!/usr/bin/env bash
set -euo pipefail

# 本文件用于启动rerun

# 配置区

RBL_FILE_NAME="${RBL_FILE_NAME:-nv_dual.rbl}" # 双视图蓝图，相对当前脚本
RERUN_MEMORY_LIMIT="${RERUN_MEMORY_LIMIT:-512MB}"
RERUN_SERVER_MEMORY_LIMIT="${RERUN_SERVER_MEMORY_LIMIT:-128MB}"
RERUN_PORT="${RERUN_PORT:-9876}"
RERUN_BIN="${RERUN_BIN:-rerun}"

# 非配置区

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RBL_FILE="$SCRIPT_DIR/$RBL_FILE_NAME"

if [[ ! -f "$RBL_FILE" ]]; then
  echo "找不到蓝图：$RBL_FILE。请先运行 generate_blueprint.py。" >&2
  exit 1
fi

# 0.37 使用位置参数加载 .rbl；仅加载蓝图时仍启动 SDK 接收服务。
exec "$RERUN_BIN" --memory-limit "$RERUN_MEMORY_LIMIT" \
  --server-memory-limit "$RERUN_SERVER_MEMORY_LIMIT" \
  --port "$RERUN_PORT" "$RBL_FILE" "$@"
