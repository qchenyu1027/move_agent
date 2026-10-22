"""默认启动实时网页；保留 --cli 兼容终端模式。"""
import argparse
from pathlib import Path
import uuid
from config.settings import DEFAULT_MAX_STEPS, RUNS_DIR
from harness.runtime import Harness
from harness.model import ChatClient
from robot.client import RobotClient
from robot.http_client import RealRobotClient
from tools import ROBOT_TOOL_SCHEMAS
from web import run_web


def run_cli(request, max_steps, mode):
    backend = RealRobotClient() if mode == "robot" else RobotClient()
    tools = ROBOT_TOOL_SCHEMAS if mode == "robot" else None
    prompt = "prompt_robot.md" if mode == "robot" else "prompt.md"
    harness = Harness(ChatClient(tools), backend, Path(RUNS_DIR) / (uuid.uuid4().hex + ".jsonl"), max_steps, prompt)
    if mode == "robot":
        print("真实机器人模式已启动。所有动作将发送到远程机器人 API。")
        print(backend.health())
    else:
        print("虚拟小球游戏已启动（● 小球 / ■ 障碍 / ◎ 终点）")
        print(backend.observe_game()["board"])
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
            if mode == "virtual":
                print(backend.observe_game()["board"])


def main():
    parser = argparse.ArgumentParser(description="LLM 控制虚拟环境或真实机器人")
    parser.add_argument("request", nargs="?")
    parser.add_argument("--cli", action="store_true", help="使用旧的终端界面")
    parser.add_argument("--mode", choices=["virtual", "robot"], default="virtual")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--max-steps", type=int, default=DEFAULT_MAX_STEPS)
    args = parser.parse_args()
    if not 1 <= args.max_steps <= 100:
        parser.error("--max-steps 必须在1到100之间")
    if args.cli or args.request:
        run_cli(args.request, args.max_steps, args.mode)
    else:
        run_web(args.host, args.port, args.max_steps, args.mode)


if __name__ == "__main__":
    main()
