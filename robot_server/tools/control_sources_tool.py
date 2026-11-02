# 用途：封装 ROS IK 与关节命令源切换脚本；运行：由 agent_tools.scheduler 调用。
"""ROS 控制源切换工具封装。"""

from tools.common import option_arguments, run_command, script_command


CONTROL_SOURCE_OPTIONS = {'namespace', 'wait-timeout-s'}


class ControlSourcesTool:
    """先选择关节命令源，再选择 IK 请求源。"""

    def build_command(self, task):
        unknown = set(task) - {'tool', 'ik_source', 'joint_source', 'options'}
        if unknown:
            raise ValueError(f'control_sources 存在未知字段：{sorted(unknown)}')
        for name in ('ik_source', 'joint_source'):
            value = task.get(name)
            if type(value) is not int or value < 0:
                raise ValueError(f'{name} 必须是非负整数')
        return (script_command('select_control_sources.py') +
                [f'--ik-source={task["ik_source"]}',
                 f'--joint-source={task["joint_source"]}'] +
                option_arguments(task.get('options', {}), CONTROL_SOURCE_OPTIONS))

    def run(self, task):
        """显式调用此方法才切换真实机器人控制源。"""
        return run_command(self.build_command(task))
