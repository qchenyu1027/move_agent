#!/usr/bin/python3
# 用途：离线验证底盘世界位姿移动脚本的输入、定位预检和停止条件。
# 运行：python3 -B -m unittest -v agent_tools.test_base_move_world
"""不连接 VLAHost 或底盘的控制流程测试。"""

import contextlib
import io
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

# 经 tools.move_tool 加载 tools/scripts 下的副本，避免误测根目录同名原件。
from tools.move_tool import base_move_world


def make_pose(yaw=0.0, stamp=1.0, age=0.01):
    """构造符合 VLAHost 返回格式的世界底盘位姿。"""
    return {
        'valid': True,
        'frame_id': 'camera_init',
        'child_frame_id': 'body',
        'received_age_s': age,
        'stamp_s': stamp,
        'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'yaw_rad': yaw,
    }


class FakeClock:
    """让等待逻辑在测试中立即推进虚拟时间。"""

    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class BaseMoveWorldTest(unittest.TestCase):
    def test_seven_pose_values_convert_quaternion_to_yaw(self):
        values = [
            1.1593247946463563, -0.0606813495159879,
            0.0006836203517147192, -0.003125499995759508,
            0.00028895241985152706, 0.1580279854321458,
            0.9874296448742731,
        ]
        x, y, z, yaw = base_move_world.pose_values_to_target(values)
        self.assertEqual((x, y, z), tuple(values[:3]))
        self.assertAlmostEqual(yaw, 0.317383, places=6)

    def test_stable_pose_requires_repeated_source_stamps(self):
        clock = FakeClock()
        mover = base_move_world.BaseWorldMover(SimpleNamespace(
            pose_ready_timeout_s=3.0, pose_stable_s=2.0))

        def get_pose():
            pose = make_pose(stamp=1.0 + math.floor(clock.now * 10.0) / 10.0)
            base_move_world.validate_base_world_pose(pose)
            return pose

        mover.get_pose = get_pose
        with patch.object(base_move_world, 'time', clock):
            result = mover.wait_for_fresh_pose()
        self.assertGreaterEqual(clock.now, 2.0)
        self.assertGreater(result['stamp_s'], 1.0)

    def test_sporadic_pose_never_starts_motion(self):
        clock = FakeClock()
        mover = base_move_world.BaseWorldMover(SimpleNamespace(
            pose_ready_timeout_s=3.0, pose_stable_s=2.0))

        def get_pose():
            pose = make_pose(
                stamp=1.0 + math.floor(clock.now),
                age=clock.now % 1.0)
            base_move_world.validate_base_world_pose(pose)
            return pose

        mover.get_pose = get_pose
        with patch.object(base_move_world, 'time', clock):
            with contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError, '持续更新超时'):
                    mover.wait_for_fresh_pose()
        self.assertFalse(mover.command_attempted)

    def test_republished_old_stamp_does_not_pass_preflight(self):
        clock = FakeClock()
        mover = base_move_world.BaseWorldMover(SimpleNamespace(
            pose_ready_timeout_s=3.0, pose_stable_s=2.0))
        mover.get_pose = lambda: make_pose(stamp=1.0, age=0.01)
        with patch.object(base_move_world, 'time', clock):
            with self.assertRaisesRegex(RuntimeError, '持续更新超时'):
                mover.wait_for_fresh_pose()
        self.assertFalse(mover.command_attempted)

    def test_check_only_never_sends_motion_request(self):
        clock = FakeClock()
        args = SimpleNamespace(
            check_only=True, vlahost_url='http://127.0.0.1:8011',
            http_timeout_s=3.0, pose_ready_timeout_s=3.0,
            pose_stable_s=2.0)
        paths = []

        def read_only_request(_url, path, body=None, timeout=3.0):
            self.assertIsNone(body)
            paths.append(path)
            return {'pose': make_pose(stamp=1.0 + math.floor(clock.now * 10.0) / 10.0)}

        with patch.object(base_move_world, 'time', clock), \
                patch.object(base_move_world, 'parse_args', return_value=args), \
                patch.object(base_move_world, 'request_json', read_only_request):
            with contextlib.redirect_stdout(io.StringIO()):
                result = base_move_world.main()
        self.assertEqual(result, 0)
        self.assertTrue(paths)
        self.assertEqual(set(paths), {'/base_world_pose'})

    def test_unreachable_z_fails_before_translation(self):
        mover = base_move_world.BaseWorldMover(SimpleNamespace(
            target_z_m=0.1, position_tolerance_m=0.05))
        mover.wait_for_fresh_pose = lambda: make_pose()
        calls = []
        mover.move_xy = lambda: calls.append(True)
        with self.assertRaisesRegex(RuntimeError, '不能主动调整 z'):
            mover.run()
        self.assertFalse(calls)

    def test_default_odometry_interruption_does_not_retry(self):
        mover = base_move_world.BaseWorldMover(SimpleNamespace(odometry_retries=0))
        calls = []

        def interrupted():
            calls.append(True)
            raise base_move_world.OdometryInterruptedError('stale')

        mover.move_xy_once = interrupted
        with self.assertRaisesRegex(RuntimeError, '未自动继续'):
            mover.move_xy()
        self.assertEqual(len(calls), 1)

    def test_segment_stays_below_server_limit(self):
        mover = base_move_world.BaseWorldMover(SimpleNamespace(
            target_x_m=1.0, target_y_m=0.0, position_tolerance_m=0.05))
        mover.wait_for_fresh_pose = lambda: make_pose()
        lengths = []

        class SegmentCaptured(Exception):
            pass

        def capture(dx, dy, _x, _y, _label):
            lengths.append(math.hypot(dx, dy))
            raise SegmentCaptured

        mover.send_and_wait_xy = capture
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SegmentCaptured):
                mover.move_xy_segmented()
        self.assertAlmostEqual(lengths[0], 0.49)
        self.assertLess(lengths[0], 0.50)

    def test_world_length_error_accepts_new_server_limit_message(self):
        self.assertTrue(base_move_world.BaseWorldMover.is_server_world_length_error(
            ValueError('world offset length must be 0.05..2.00 m')))

    def test_yaw_feedback_opposite_command_aborts(self):
        clock = FakeClock()
        mover = base_move_world.BaseWorldMover(SimpleNamespace(
            target_yaw_rad=0.5, yaw_tolerance_rad=0.1,
            yaw_timeout_s=5.0, yaw_speed_rad_s=0.2,
            yaw_command_duration_s=0.2))
        mover.wait_for_fresh_pose = lambda: make_pose(yaw=0.0, stamp=1.0)
        mover.send_yaw_velocity = lambda _speed: {'success': True, 'execution': 'active'}
        mover.wait_for_next_pose = lambda _stamp, _deadline: make_pose(yaw=-0.04, stamp=2.0)
        with patch.object(base_move_world, 'time', clock):
            with contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError, '方向与速度命令相反'):
                    mover.move_yaw()

    def test_yaw_reaches_target_after_fresh_feedback(self):
        clock = FakeClock()
        mover = base_move_world.BaseWorldMover(SimpleNamespace(
            target_yaw_rad=0.12, yaw_tolerance_rad=0.1,
            yaw_timeout_s=5.0, yaw_speed_rad_s=0.2,
            yaw_command_duration_s=0.2))
        mover.wait_for_fresh_pose = lambda: make_pose(yaw=0.0, stamp=1.0)
        speeds = []
        mover.send_yaw_velocity = lambda speed: (
            speeds.append(speed) or {'success': True, 'execution': 'active'})
        mover.wait_for_next_pose = lambda _stamp, _deadline: make_pose(yaw=0.04, stamp=2.0)
        mover.stop_yaw = lambda: True
        with patch.object(base_move_world, 'time', clock):
            with contextlib.redirect_stdout(io.StringIO()):
                mover.move_yaw()
        self.assertEqual(len(speeds), 1)
        self.assertGreater(speeds[0], 0.0)

    def test_motion_error_sends_stop_commands(self):
        calls = []

        def fail():
            raise RuntimeError('反馈异常')

        mover = SimpleNamespace(
            command_attempted=True,
            run=fail,
            cancel_translation=lambda: calls.append('cancel'),
            stop_yaw=lambda: calls.append('zero'))
        with patch.object(base_move_world, 'parse_args', return_value=SimpleNamespace(
                check_only=False)), \
                patch.object(base_move_world, 'BaseWorldMover', return_value=mover):
            with contextlib.redirect_stderr(io.StringIO()):
                result = base_move_world.main()
        self.assertEqual(result, 1)
        self.assertEqual(calls, ['cancel', 'zero'])


if __name__ == '__main__':
    unittest.main()
