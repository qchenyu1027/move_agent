#!/usr/bin/env bash
set -euo pipefail

ROBOT_API_BASE="${ROBOT_API_BASE:-http://10.121.40.94:8000}"

curl -4 --noproxy '*' -fsS \
  --connect-timeout "${ROBOT_HEALTH_TIMEOUT_S:-15}" \
  --max-time "${ROBOT_HEALTH_TIMEOUT_S:-15}" \
  "$ROBOT_API_BASE/health" | python3 -m json.tool
