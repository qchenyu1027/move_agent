# 用途：把左右臂 ROS 笛卡尔遥操接入任务调度器；运行：由 agent_tools.scheduler 调用。
"""左右臂笛卡尔遥操工具封装。"""

from tools.common import option_arguments, run_command, script_command


ARM_TELEOP_OPTIONS = {
    'imp-cart', 'incremental', 'namespace', 'position-step-mm',
    'angle-step-deg', 'repeat-hz', 'max-position-offset-mm',
    'max-angle-offset-deg', 'feedback-timeout', 'rate-hz', 'ik-source',
    'joint-source', 'head-joint-count', 'gripper-close-value',
    'gripper-open-value', 'gripper-topic', 'model-xml', 'control-backend',
    'vlahost-url', 'move-x-mm', 'move-y-mm', 'move-z-mm',
    'once-timeout-s', 'once-tolerance-mm', 'once-settle-s',
}


class ArmTeleopTool:
    """按 side 启动 ROS 交互遥操或一次性笛卡尔位移。"""

    def build_command(self, task):
        unknown = set(task) - {'tool', 'side', 'options'}
        if unknown:
            raise ValueError(f'arm_teleop 存在未知字段：{sorted(unknown)}')
        side = task.get('side')
        if side not in ('left', 'right'):
            raise ValueError('side 必须是 left 或 right')
        options = task.get('options', {})
        return (script_command('arm_teleop.py')
                + [f'--side={side}', '--execute']
                + option_arguments(
                    options, ARM_TELEOP_OPTIONS,
                    flags={'imp-cart', 'incremental'}))

    def run(self, task):
        """显式调用此方法才启动实际 ROS 遥操。"""
        return run_command(self.build_command(task))
