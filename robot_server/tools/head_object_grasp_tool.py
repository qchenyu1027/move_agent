# 用途：把头部视觉物体抓取脚本接入 agent_tools 调度器；运行：由 agent_tools.scheduler 或 API 调用。
"""头部视觉物体抓取工具封装。"""

import math

from tools.common import option_arguments, run_command, script_command


HEAD_OBJECT_GRASP_OPTIONS = {
    'position-only', 'reach-only', 'control-backend', 'vlahost-url', 'http-timeout-s', 'http-action-hz',
    'http-log-hz', 'namespace', 'model-xml', 'ik-topic', 'ik-source',
    'joint-source', 'capture-timeout', 'preflight-timeout', 'motion-timeout',
    'feedback-timeout', 'max-pose-age', 'stable-samples',
    'stable-position-mm', 'stable-angle-deg', 'approach-height-mm',
    'lift-height-mm', 'max-displacement-mm', 'max-target-angle-deg',
    'position-tolerance-mm', 'angle-tolerance-deg',
    'approach-position-tolerance-mm', 'approach-angle-tolerance-deg', 'settle-time',
    'gripper-duration', 'gripper-rate-hz',
}


class HeadObjectGraspTool:
    """接收视觉物体位姿，移动指定手臂并完成抓取。"""

    def build_command(self, task):
        allowed_fields = {
            'tool', 'side', 'object_pose_topic', 'pose_request_topic',
            'grasp_offset_mm', 'options',
        }
        unknown = set(task) - allowed_fields
        if unknown:
            raise ValueError(f'head_object_grasp 存在未知字段：{sorted(unknown)}')
        side = task.get('side')
        if side not in ('left', 'right'):
            raise ValueError('side 必须是 left 或 right')

        object_topic = task.get('object_pose_topic', '/object_pose')
        if not isinstance(object_topic, str) or not object_topic:
            raise ValueError('object_pose_topic 必须是非空字符串')
        request_topic = task.get('pose_request_topic', '/f2g_pose/request')
        if request_topic is not None and (
                not isinstance(request_topic, str) or not request_topic):
            raise ValueError('pose_request_topic 必须是非空字符串或 null')

        offset = task.get('grasp_offset_mm', (0.0, 0.0, 0.0))
        if (not isinstance(offset, (list, tuple)) or len(offset) != 3 or
                any(isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) for value in offset)):
            raise ValueError('grasp_offset_mm 必须是 3 个有限数值')

        command = script_command('head_object_grasp.py') + [
            f'--side={side}', f'--object-pose-topic={object_topic}', '--execute',
            '--grasp-offset-mm', *[str(value) for value in offset],
        ]
        if request_topic is not None:
            command.append(f'--pose-request-topic={request_topic}')
        return command + option_arguments(
            task.get('options', {}), HEAD_OBJECT_GRASP_OPTIONS,
            flags={'position-only', 'reach-only'})

    def run(self, task):
        """显式调用此方法才执行实际抓取。"""
        return run_command(self.build_command(task))
