# 用途：封装 ROS go home 控制脚本并接入任务调度器；运行：由 agent_tools.scheduler 调用。
"""ROS go home 工具封装。"""

from tools.common import option_arguments, run_command, script_command


GO_HOME_OPTIONS = {'namespace', 'service', 'wait-timeout-s', 'completion-margin-s'}


class GoHomeTool:
    """调用 ``control/go_home`` ROS Trigger 服务。"""

    def build_command(self, task):
        unknown = set(task) - {'tool', 'options'}
        if unknown:
            raise ValueError(f'go_home 存在未知字段：{sorted(unknown)}')
        return script_command('go_home.py') + option_arguments(
            task.get('options', {}), GO_HOME_OPTIONS)

    def run(self, task):
        """显式调用此方法才执行真实机器人动作。"""
        return run_command(self.build_command(task))
