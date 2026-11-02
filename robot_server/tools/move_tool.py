# 用途：封装默认位姿列表与显式世界位姿移动；运行：python3 -m agent_tools.scheduler --help。
"""构建底盘原生世界坐标导航任务命令。"""

import math
import sys

from tools.common import SCRIPTS_DIR, option_arguments, run_command, script_command

# 位姿数据为纯 Python 模块，预览不需要加载 ROS。
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
import native_nav_targets


MOVE_OPTIONS = {
    'vlahost-url', 'speed-m-s', 'release-parking', 'service-timeout',
    'http-timeout-s', 'pose-ready-timeout-s', 'pose-stable-s',
    'move-timeout-s', 'position-tolerance-m', 'yaw-tolerance-rad',
    'yaw-reference-rad', 'yaw-max-travel-rad', 'max-attempts', 'pause-s',
}


class MoveTool:
    """fixed 为脚本默认目标；pose_ids 为预设编号；pose 为七维世界位姿。"""

    def build_command(self, task):
        unknown = set(task) - {'tool', 'fixed', 'pose_ids', 'pose', 'options'}
        if unknown:
            raise ValueError(f'move 存在未知字段：{sorted(unknown)}')
        targets = ('fixed', 'pose_ids', 'pose')
        if sum(name in task for name in targets) != 1:
            raise ValueError('move 必须且只能指定 fixed、pose_ids 或 pose')
        options = task.get('options', {})
        if 'fixed' in task:
            if task['fixed'] is not True:
                raise ValueError('fixed 必须为 true')
            command = script_command('native_nav_test.py') + ['--fixed']
            allowed = MOVE_OPTIONS
        elif 'pose_ids' in task:
            ids = task['pose_ids']
            if not isinstance(ids, list):
                raise ValueError('pose_ids 必须是非空整数列表')
            native_nav_targets.parse_move_list(repr(ids))
            for index in ids:
                native_nav_targets.pose_to_target(
                    native_nav_targets.DEFAULT_POSES[index - 1], index)
            command = script_command('native_nav_test.py') + ['--pose-ids', repr(ids)]
            allowed = MOVE_OPTIONS | {'pause-s'}
        else:
            pose = task['pose']
            if (not isinstance(pose, list) or len(pose) != 7
                    or any(isinstance(v, bool) or not isinstance(v, (int, float))
                           or not math.isfinite(v) for v in pose)):
                raise ValueError('pose 必须是 7 个有限数值：x y z qx qy qz qw')
            native_nav_targets.quaternion_values_to_target(pose)
            command = script_command('native_nav_test.py') + ['--pose-quat', *map(str, pose)]
            allowed = MOVE_OPTIONS
        return (command + option_arguments(options, allowed, flags={'release-parking'})
                + ['--wait', '--execute'])

    def run(self, task):
        """显式调用此方法即执行真实移动。"""
        return run_command(self.build_command(task))
