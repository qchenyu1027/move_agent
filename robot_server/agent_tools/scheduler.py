# 用途：按 JSON 列表串行调度 move/grasp，失败即停止；运行：python3 -m agent_tools.scheduler agent_tools/tasks.example.json。
"""默认仅预览命令，添加 --execute 才启动控制脚本。"""

import argparse
import json
from pathlib import Path
import shlex
import sys
import threading

# 同时支持从其他工作目录通过脚本绝对路径启动。
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.common import run_command
from tools.grasp_tool import GraspTool
from tools.move_tool import MoveTool


class TaskScheduler:
    """先校验整个任务列表，再按顺序执行；不自动重试机器人动作。"""

    def __init__(self):
        self.tools = {'move': MoveTool(), 'grasp': GraspTool()}
        self.cancel_event = threading.Event()

    def reset_cancel(self):
        self.cancel_event.clear()

    def cancel_current(self):
        self.cancel_event.set()

    def prepare(self, tasks):
        if not isinstance(tasks, list) or not tasks:
            raise ValueError('任务计划必须是非空 JSON 数组')
        commands = []
        for index, task in enumerate(tasks, 1):
            if not isinstance(task, dict) or not isinstance(task.get('tool'), str):
                raise ValueError(f'任务 {index} 必须是包含 tool 的对象')
            tool = self.tools.get(task['tool'])
            if tool is None:
                raise ValueError(f'任务 {index} 的 tool 只能是 move 或 grasp')
            try:
                commands.append(tool.build_command(task))
            except ValueError as error:
                raise ValueError(f'任务 {index}：{error}') from error
        return commands

    def run(self, tasks, *, execute=False):
        commands = self.prepare(tasks)
        for index, command in enumerate(commands, 1):
            print(f'[{index}/{len(commands)}] '
                  f'{"执行" if execute else "预览"}：{shlex.join(command)}', flush=True)
            if execute:
                code = run_command(command, self.cancel_event)
                if code != 0:
                    print(f'任务 {index} 失败（退出码 {code}），停止后续任务。',
                          file=sys.stderr)
                    return code if code > 0 else 128 - code
        print('全部任务执行完成。' if execute else '预览完成，未启动任何控制脚本。')
        return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan', type=Path, help='JSON 任务数组文件')
    parser.add_argument('--execute', action='store_true', help='实际执行机器人动作')
    args = parser.parse_args(argv)
    try:
        with args.plan.open(encoding='utf-8') as stream:
            tasks = json.load(stream)
        return TaskScheduler().run(tasks, execute=args.execute)
    except (OSError, ValueError) as error:
        print(f'调度失败：{error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    sys.exit(main())
