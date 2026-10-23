#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST="${MOVE_AGENT_HOST:-0.0.0.0}"
PORT="${MOVE_AGENT_PORT:-8080}"
MAX_STEPS="${MOVE_AGENT_MAX_STEPS:-40}"

cd "$PROJECT_DIR"
exec python3 app.py \
  --mode virtual \
  --host "$HOST" \
  --port "$PORT" \
  --max-steps "$MAX_STEPS"
