# 用途：先到独立保存的原始双手水平 home，再只调整躯干升降；运行：由 agent_tools.scheduler 调用。
"""封装原始 home + 躯干 IK 高度工具。"""

import math

from tools.common import option_arguments, run_command, script_command


TORSO_OPTIONS = {
    'namespace', 'home-qpos', 'model-xml', 'move-timeout-s', 'command-hz',
    'source-index', 'max-offset-mm', 'position-tolerance-rad', 'joint-log-hz',
    'ramp-speed-rad-s', 'min-ramp-s',
}


class TorsoTool:
    """使用独立原始 home；高度阶段只调整 6 个躯干关节，双臂保持 home。

    高度 IK 保持躯干左右位置、Z 高度和姿态，放开前后位置约束。
    """

    def build_command(self, task):
        unknown = set(task) - {'tool', 'offset_mm', 'down_mm', 'options'}
        if unknown:
            raise ValueError(f'torso_home_down 存在未知字段：{sorted(unknown)}')
        if 'offset_mm' in task and 'down_mm' in task:
            raise ValueError('offset_mm 与 down_mm 只能二选一')
        if 'offset_mm' in task:
            offset_mm = task['offset_mm']
        elif 'down_mm' in task:
            # 兼容旧任务格式：down_mm 始终表示向下。
            down_mm = task['down_mm']
            if isinstance(down_mm, bool) or not isinstance(down_mm, (int, float)):
                raise ValueError('down_mm 必须是有限数值（单位 mm）')
            offset_mm = -down_mm
        else:
            raise ValueError('必须指定 offset_mm（负数下降、正数上升）')
        if (isinstance(offset_mm, bool) or not isinstance(offset_mm, (int, float))
                or not math.isfinite(offset_mm) or offset_mm == 0.0):
            raise ValueError(
                'offset_mm 必须是非零有限数值（单位 mm）；幅度由躯干 IK 收敛和关节限位约束')
        options = task.get('options', {})
        command = script_command('torso_home_down.py') + [
            f'--offset-mm={offset_mm}',
        ]
        return command + option_arguments(options, TORSO_OPTIONS)

    def run(self, task):
        """显式调用此方法才执行真实机器人动作。"""
        return run_command(self.build_command(task))
