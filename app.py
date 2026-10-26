"""真实机器人 LLM 控制入口；默认启动实时网页。"""
import argparse
from pathlib import Path
import uuid

from config.settings import RUNS_DIR
from harness.model import ChatClient
from harness.runtime import Harness
from robot.http_client import RealRobotClient
from tools import ROBOT_TOOL_SCHEMAS
from web import run_web


def build_harness():
    return Harness(
        ChatClient(ROBOT_TOOL_SCHEMAS),
        RealRobotClient(),
        Path(RUNS_DIR) / (uuid.uuid4().hex + ".jsonl"),
    )


def run_cli(request):
    harness = build_harness()
    print("真实机器人模式已启动。动作前会自动检查机器人状态。")
    if request:
        print(harness.run(request))
        return
    while True:
        try:
            request = input("\n你：").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if request.lower() in {"exit", "quit", "退出"}:
            break
        if request:
            print("助手：" + harness.run(request))


def main():
    parser = argparse.ArgumentParser(description="LLM 控制真实 Luna 机器人")
    parser.add_argument("request", nargs="?")
    parser.add_argument("--cli", action="store_true", help="使用终端界面")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    if args.cli or args.request:
        run_cli(args.request)
    else:
        run_web(args.host, args.port)


if __name__ == "__main__":
    main()
