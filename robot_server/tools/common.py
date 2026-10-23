# 用途：构建工具参数并执行独立控制进程；运行：python3 -m agent_tools.scheduler --help。
"""工具共用的参数校验和进程执行。"""

import math
from pathlib import Path
import signal
import subprocess
import sys


# 控制脚本及其数据（标定偏移、位姿快照）都在 tools/scripts/，并以该目录为工作目录运行。
SCRIPTS_DIR = Path(__file__).resolve().parent / 'scripts'


def option_arguments(options, allowed):
    """只接受完整选项名，避免 argparse 缩写或控制开关改变任务语义。"""
    if not isinstance(options, dict):
        raise ValueError('options 必须是对象')
    arguments = []
    for name, value in options.items():
        if name not in allowed:
            raise ValueError(f'不支持的选项 {name!r}；允许：{", ".join(sorted(allowed))}')
        if (isinstance(value, bool) or not isinstance(value, (str, int, float))
                or isinstance(value, float) and not math.isfinite(value)
                or isinstance(value, str) and (not value or '\x00' in value)):
            raise ValueError(f'选项 {name} 必须是非空字符串或有限数值')
        # 使用等号绑定值，避免字符串值被误认为另一个命令行选项。
        arguments.append(f'--{name}={value}')
    return arguments


def script_command(name):
    """沿用当前 Python 和已加载的 ROS 环境，不通过 shell 执行。"""
    return [sys.executable, '-u', str(SCRIPTS_DIR / name)]


def run_command(command, cancel_event=None):
    """等待控制脚本结束；中断时先让原脚本执行停止和清理逻辑。"""
    process = subprocess.Popen(command, cwd=SCRIPTS_DIR)
    cancel_sent = False
    try:
        while True:
            try:
                if cancel_event is None:
                    return process.wait()
                return process.wait(timeout=0.1)
            except subprocess.TimeoutExpired:
                if cancel_event.is_set() and not cancel_sent:
                    process.send_signal(signal.SIGINT)
                    cancel_sent = True
    except KeyboardInterrupt:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        # 不强杀控制进程，以免中断原脚本中的停止指令。
        while True:
            try:
                process.wait()
                break
            except KeyboardInterrupt:
                continue
        return 130
