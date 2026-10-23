# 用途：封装 AprilTag 左手抓放及回位流程；运行：python3 -m agent_tools.scheduler --help。
"""通过独立进程加载 ROS，离线预览不导入 ROS 依赖。"""

from tools.common import option_arguments, run_command, script_command


GRASP_OPTIONS = {
    'tag-size', 'offset-config', 'calibration-file', 'intrinsics-file',
    'image-topic', 'capture-timeout', 'max-target-angle-deg',
    'position-tolerance-mm', 'angle-tolerance-deg', 'timeout',
    'max-displacement-mm', 'feedback-timeout', 'preflight-timeout',
    'settle-time', 'namespace', 'gripper-duration', 'gripper-rate-hz',
    'gripper-topic', 'gripper-feedback-topic', 'gripper-status-topic',
    'stability-window-sec', 'stability-position-jitter-mm',
    'stability-angle-jitter-deg', 'return-speed-mm-s', 'return-pose-file',
    'approach-height-mm', 'approach-step-mm', 'base-right-distance-mm',
    'base-lateral-speed-m-s', 'base-return-world-x-m', 'base-return-world-y-m',
    'base-return-world-z-m', 'base-return-world-yaw-rad',
    'base-return-yaw-speed-rad-s', 'base-return-pose-tolerance-m',
    'base-return-yaw-tolerance-rad', 'vlahost-url',
}


class GraspTool:
    """成功表示原抓放、底盘回位和 home 流程全部完成。"""

    def build_command(self, task):
        unknown = set(task) - {'tool', 'tag_id', 'grasp_and_lift', 'options'}
        if unknown:
            raise ValueError(f'grasp 存在未知字段：{sorted(unknown)}')
        tag_id = task.get('tag_id', 5)
        if isinstance(tag_id, bool) or not isinstance(tag_id, int) or tag_id < 0:
            raise ValueError('tag_id 必须是非负整数')
        grasp_and_lift = task.get('grasp_and_lift', False)
        if not isinstance(grasp_and_lift, bool):
            raise ValueError('grasp_and_lift 必须是布尔值')
        mode = ['--grasp-and-lift'] if grasp_and_lift else []
        return (script_command('apriltag_left_tool_grasp_test.py')
                + [f'--tag-id={tag_id}', '--execute'] + mode
                + option_arguments(task.get('options', {}), GRASP_OPTIONS))

    def run(self, task):
        """显式调用此方法即执行真实抓放。"""
        return run_command(self.build_command(task))
