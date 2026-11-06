# 用途：将指定手臂末端沿 base_link 做一次有限短位移；运行：由 agent_tools.scheduler 调用。
"""把单侧 EEF 的有限位移封装为任务调度器工具。"""

import math

from tools.common import option_arguments, run_command, script_command


EEF_DELTA_OPTIONS = {
    'control-backend', 'vlahost-url', 'namespace', 'feedback-timeout',
    'rate-hz', 'ik-source', 'joint-source', 'head-joint-count', 'model-xml',
    'once-timeout-s', 'once-tolerance-mm', 'once-settle-s',
}


class EefDeltaTool:
    """让一侧手臂的末端沿 base_link 做一次受限平移。"""

    def build_command(self, task):
        unknown = set(task) - {
            'tool', 'side', 'dx_mm', 'dy_mm', 'dz_mm', 'options',
        }
        if unknown:
            raise ValueError(f'eef_delta 存在未知字段：{sorted(unknown)}')
        side = task.get('side')
        if side not in ('left', 'right'):
            raise ValueError('side 必须是 left 或 right')
        values = tuple(task.get(name) for name in ('dx_mm', 'dy_mm', 'dz_mm'))
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for value in values
        ):
            raise ValueError('dx_mm、dy_mm、dz_mm 必须是有限数值')
        if not any(values):
            raise ValueError('eef_delta 至少需要一个非零位移')
        if math.dist(values, (0.0, 0.0, 0.0)) > 200.0:
            raise ValueError('eef_delta 位移向量范数不能超过 200 mm')

        options = dict(task.get('options', {}))
        options.setdefault('control-backend', 'vlahost')
        options.setdefault('max-position-offset-mm', 200)
        allowed = EEF_DELTA_OPTIONS | {'max-position-offset-mm'}
        command = script_command('arm_teleop.py') + [
            f'--side={side}', '--execute',
            f'--move-x-mm={values[0]}',
            f'--move-y-mm={values[1]}',
            f'--move-z-mm={values[2]}',
        ] + option_arguments(options, allowed)
        return command

    def run(self, task):
        """执行一次受限 EEF 平移。"""
        return run_command(self.build_command(task))
