# 用途：构建左右夹爪开合 ROS 控制命令；运行：由 agent_tools.scheduler 调用。
"""左右夹爪开合工具封装。"""

import math

from tools.common import option_arguments, run_command, script_command


GRIPPER_OPTIONS = {
    'open-position', 'close-position', 'speed', 'publish-hz',
    'feedback-timeout-s', 'start-position', 'hold-s', 'namespace', 'topic',
    'feedback-topic', 'control-backend', 'vlahost-url',
}


class GripperTool:
    """通过 ROS Float32 话题将单侧夹爪平滑移动到目标开合度。"""

    def build_command(self, task):
        unknown = set(task) - {'tool', 'side', 'action', 'position', 'options'}
        if unknown:
            raise ValueError(f'gripper 存在未知字段：{sorted(unknown)}')
        side = task.get('side')
        if side not in ('left', 'right'):
            raise ValueError('side 必须是 left 或 right')
        action = task.get('action')
        position = task.get('position')
        if action not in ('open', 'close', 'set'):
            raise ValueError('action 必须是 open、close 或 set')
        if action == 'set' and position is None:
            raise ValueError('action=set 时必须指定 position')
        if position is not None:
            if (isinstance(position, bool) or not isinstance(position, (int, float))
                    or not math.isfinite(position) or not 0.0 <= position <= 1.0):
                raise ValueError('position 必须是 0..1 范围内的有限数值')
        command = script_command('gripper_control.py') + [f'--side={side}']
        if action == 'open':
            command.append('--open' if position is None else f'--target-position={position}')
        elif action == 'close':
            command.append('--close' if position is None else f'--target-position={position}')
        else:
            command.append(f'--target-position={position}')
        return command + option_arguments(task.get('options', {}), GRIPPER_OPTIONS)

    def run(self, task):
        """显式调用此方法才执行真实夹爪动作。"""
        return run_command(self.build_command(task))
