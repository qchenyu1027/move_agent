#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST="${MOVE_AGENT_HOST:-0.0.0.0}"
PORT="${MOVE_AGENT_PORT:-8080}"
MAX_STEPS="${MOVE_AGENT_MAX_STEPS:-40}"

export ROBOT_API_BASE="${ROBOT_API_BASE:-http://10.121.40.94:8000}"
export ROBOT_HEALTH_TIMEOUT_S="${ROBOT_HEALTH_TIMEOUT_S:-15}"
export ROBOT_ACTION_TIMEOUT_S="${ROBOT_ACTION_TIMEOUT_S:-600}"

ROBOT_HOST="${ROBOT_API_BASE#*://}"
ROBOT_HOST="${ROBOT_HOST%%:*}"
export NO_PROXY="${NO_PROXY:+$NO_PROXY,}$ROBOT_HOST"

if [[ -z "${ROBOT_API_KEY:-}" ]]; then
  echo "错误：未设置 ROBOT_API_KEY。" >&2
  echo "请先执行：read -s -p 'ROBOT_API_KEY: ' ROBOT_API_KEY; echo; export ROBOT_API_KEY" >&2
  exit 2
fi

cd "$PROJECT_DIR"
exec python3 app.py \
  --mode robot \
  --host "$HOST" \
  --port "$PORT" \
  --max-steps "$MAX_STEPS"
