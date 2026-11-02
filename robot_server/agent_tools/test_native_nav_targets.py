# 用途：离线验证世界位姿到机体相对动作的换算；运行：python3 -B -m unittest -v agent_tools.test_native_nav_targets。
"""不连接底盘，检查机体坐标旋转和动作幅度。"""

import math
import sys
import unittest
from pathlib import Path

# 允许从项目根目录直接执行该测试文件，而不依赖当前工作目录。
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.scripts.native_nav_targets import relative_step, unwrap_near, world_to_body_delta


class RelativeStepTest(unittest.TestCase):
    def test_world_difference_rotates_with_current_yaw(self):
        dx, dy, turn, distance = relative_step(
            (1.0, 2.0, 0.0, math.pi / 2),
            (1.0, 2.2, 0.0, math.pi / 2), max_step_m=0.25)
        self.assertAlmostEqual(dx, 0.2)
        self.assertAlmostEqual(dy, 0.0)
        self.assertAlmostEqual(turn, 0.0)
        self.assertAlmostEqual(distance, 0.2)

    def test_sideways_motion_and_bounded_yaw(self):
        dx, dy, turn, distance = relative_step(
            (0.0, 0.0, 0.0, math.pi / 2),
            (0.2, 0.0, 0.0, -math.pi / 2))
        self.assertAlmostEqual(dx, 0.0, places=7)
        self.assertAlmostEqual(dy, -0.10)
        self.assertAlmostEqual(turn, -math.pi / 6)
        self.assertAlmostEqual(distance, 0.2)

    def test_wraparound_uses_short_turn(self):
        _, _, turn, _ = relative_step(
            (0, 0, 0, math.pi - 0.05),
            (0, 0, 0, -math.pi + 0.05))
        self.assertAlmostEqual(turn, 0.10)

    def test_invalid_pose_rejected(self):
        with self.assertRaises(ValueError):
            relative_step((float('nan'), 0, 0, 0), (1, 0, 0, 0))


class BodyFrameNavigationTest(unittest.TestCase):
    def test_world_delta_is_rotated_into_body_frame(self):
        dx, dy = world_to_body_delta(
            (1.0, 2.0, 0.0, math.pi / 2),
            (1.0, 2.2, 0.0, 0.0),
        )
        self.assertAlmostEqual(dx, 0.2)
        self.assertAlmostEqual(dy, 0.0)

    def test_equivalent_yaw_uses_nearest_reference(self):
        self.assertAlmostEqual(unwrap_near(-math.pi + 0.05, math.pi - 0.05), math.pi + 0.05)


if __name__ == '__main__':
    unittest.main()
