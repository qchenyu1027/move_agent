# 用途：封装默认位姿列表与显式世界位姿移动；运行：python3 -m agent_tools.scheduler --help。
"""复用原移动脚本，不复制底盘闭环控制。"""

import math
import sys

from tools.common import SCRIPTS_DIR, option_arguments, run_command, script_command

# 控制脚本之间按同目录模块互相导入（import base_move_world），校验时用同样方式加载。
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
import base_move_world
import base_move_world_pose_list


MOVE_OPTIONS = {
    'vlahost-url', 'speed-m-s', 'yaw-speed-rad-s', 'yaw-command-hz',
    'yaw-command-duration-s', 'position-tolerance-m', 'yaw-tolerance-rad',
    'move-timeout-s', 'yaw-timeout-s', 'http-timeout-s',
    'pose-ready-timeout-s', 'pose-stable-s', 'odometry-retries',
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
            command = script_command('base_move_world.py')
            allowed = MOVE_OPTIONS
        elif 'pose_ids' in task:
            ids = task['pose_ids']
            if not isinstance(ids, list):
                raise ValueError('pose_ids 必须是非空整数列表')
            base_move_world_pose_list.parse_move_list(repr(ids))
            for index in ids:
                base_move_world_pose_list.pose_to_target(
                    base_move_world_pose_list.DEFAULT_POSES[index - 1], index)
            command = script_command('base_move_world_pose_list.py') + [repr(ids)]
            allowed = MOVE_OPTIONS | {'pause-s'}
        else:
            pose = task['pose']
            if (not isinstance(pose, list) or len(pose) != 7
                    or any(isinstance(v, bool) or not isinstance(v, (int, float))
                           or not math.isfinite(v) for v in pose)):
                raise ValueError('pose 必须是 7 个有限数值：x y z qx qy qz qw')
            base_move_world.pose_values_to_target(pose)
            command = script_command('base_move_world.py') + [str(v) for v in pose]
            allowed = MOVE_OPTIONS
        return command + option_arguments(options, allowed)

    def run(self, task):
        """显式调用此方法即执行真实移动。"""
        return run_command(self.build_command(task))
