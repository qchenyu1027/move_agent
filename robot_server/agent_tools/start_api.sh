#!/usr/bin/env bash
# 用途：加载 Luna ROS 与私有 API 配置，并使用 ROS 兼容的 Python 启动 FastAPI；运行：agent_tools/start_api.sh。
set -eo pipefail

cd "$(dirname "$(readlink -f "$0")")/.."
source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh
set -u
set -a
source /home/liuqian/.config/robot-agent-tools/api.env
set +a
ROBOT_PYTHON="${ROBOT_PYTHON:-/usr/bin/python3}"
"$ROBOT_PYTHON" -c 'import rclpy, fastapi, uvicorn'
exec "$ROBOT_PYTHON" -B -m agent_tools.api \
  --host "${ROBOT_API_HOST:-0.0.0.0}" \
  --port "${ROBOT_API_PORT:-8000}"
