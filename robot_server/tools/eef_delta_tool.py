"""Bounded RPent-style EEF translation for the robot task scheduler."""

import math

from tools.common import option_arguments, run_command, script_command


EEF_DELTA_OPTIONS = {
    'vlahost-url', 'namespace', 'model-xml', 'tf-timeout-s',
    'motion-timeout-s', 'tolerance-mm', 'settle-s', 'max-position-offset-mm',
}


class EefDeltaTool:
    """Move one selected EEF by a bounded base_link-frame translation."""

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
        options.setdefault('max-position-offset-mm', 200)
        allowed = EEF_DELTA_OPTIONS
        command = script_command('eef_delta.py') + [
            f'--side={side}', '--execute',
            f'--dx-mm={values[0]}',
            f'--dy-mm={values[1]}',
            f'--dz-mm={values[2]}',
        ] + option_arguments(options, allowed)
        return command

    def run(self, task):
        """Execute one bounded EEF translation."""
        return run_command(self.build_command(task))
