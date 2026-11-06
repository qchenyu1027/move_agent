# 用途：离线验证 arm_teleop 一次性动作按发送时长结束的行为。
# 推荐运行：source /home/liuqian/copy/Apex_Deploy/apex_ros_env.sh && /usr/bin/python3 -B -m unittest tools.scripts.test_arm_teleop_once
"""arm_teleop 一次性位移的离线回归检查。"""

import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from right_arm_teleop import run_once


class ArmTeleopOnceTest(unittest.TestCase):
    def test_command_duration_finishes_without_eef_reaching_target(self):
        now = time.monotonic()
        logger = SimpleNamespace(info=Mock())
        node = SimpleNamespace(
            args=SimpleNamespace(execute=True, once_timeout_s=1.0,
                                 feedback_timeout=1.0, once_command_duration_s=0.05),
            lock=threading.Lock(), actual_position=(0.0, 0.0, 0.0),
            target_position=(0.0, 0.0, 0.14), initial_position=(0.0, 0.0, 0.0),
            robot_ready=True, actual_at=now, robot_state_at=now,
            joint_feedback_at=now, active=False, last_problem=None,
            first_joint_motion_command_at=None,
            select_control_backend=Mock(), readiness_problem=lambda **_: None,
            reset_target=Mock(), once_delta_mm=lambda: (0.0, 0.0, 140.0),
            translate_once=Mock(), get_logger=lambda: logger, disable=Mock())

        def enable():
            node.active = True
            node.first_joint_motion_command_at = time.monotonic()

        node.enable = enable
        with patch('right_arm_teleop.rclpy.ok', return_value=True):
            run_once(node)
        node.disable.assert_called_once_with()
        self.assertIn('未判定到位', logger.info.call_args.args[0])


if __name__ == '__main__':
    unittest.main()
