#!/usr/bin/env bash
# 用途：加载 Luna ROS 与私有 API 配置并以前台单进程启动 FastAPI；运行：agent_tools/start_api.sh。
set -eo pipefail

cd "$(dirname "$(readlink -f "$0")")/.."
source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh
set -u
set -a
source /home/liuqian/.config/robot-agent-tools/api.env
set +a
exec python3 -B -m agent_tools.api \
  --host "${ROBOT_API_HOST:-0.0.0.0}" \
  --port "${ROBOT_API_PORT:-8000}"
