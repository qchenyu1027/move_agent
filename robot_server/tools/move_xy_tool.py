# 用途：封装相对当前世界位姿的底盘 x/y 平移；运行：python3 -m agent_tools.scheduler --help。
"""构建相对当前世界坐标移动底盘 x/y 的控制命令。"""

import math

from tools.common import option_arguments, run_command, script_command


MOVE_XY_OPTIONS = {
    'vlahost-url', 'speed-m-s', 'release-parking', 'service-timeout',
    'http-timeout-s', 'pose-ready-timeout-s', 'pose-stable-s',
    'move-timeout-s', 'position-tolerance-m', 'yaw-tolerance-rad',
    'yaw-reference-rad', 'yaw-max-travel-rad', 'max-attempts', 'pause-s',
}


class MoveXYTool:
    """按米为单位，从当前世界位姿相对移动底盘 x/y。"""

    def build_command(self, task):
        unknown = set(task) - {'tool', 'x', 'y', 'x_m', 'y_m', 'options'}
        if unknown:
            raise ValueError(f'move_xy 存在未知字段：{sorted(unknown)}')
        offsets = {}
        for axis in ('x', 'y'):
            names = [name for name in (f'{axis}_m', axis) if name in task]
            if len(names) != 1:
                raise ValueError(f'move_xy 必须且只能提供 {axis}_m（或 {axis}）')
            name = names[0]
            value = task[name]
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value)):
                raise ValueError(f'{name} 必须是有限数值，单位为米')
            offsets[axis] = value
        return (script_command('native_nav_test.py')
                + [f'--offset-x-m={offsets["x"]}',
                   f'--offset-y-m={offsets["y"]}']
                + option_arguments(task.get('options', {}), MOVE_XY_OPTIONS,
                                   flags={'release-parking'})
                + ['--wait', '--execute'])

    def run(self, task):
        """显式调用此方法即执行真实底盘平移。"""
        return run_command(self.build_command(task))
