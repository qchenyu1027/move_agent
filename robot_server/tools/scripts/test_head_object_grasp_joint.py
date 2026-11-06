# 用途：离线验证头部抓取的单臂 IK、保持关节和四组 user joint 命令。
# 推荐运行：source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh && /usr/bin/python3 -B -m unittest tools.scripts.test_head_object_grasp_joint
"""head_object_grasp joint 后端的离线回归检查。"""

import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
import pinocchio as pin
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, str(Path(__file__).parent))
from head_object_grasp import HeadObjectGrasp


MODEL = str(Path(get_package_share_directory('marvin_description')) /
            'mjcf/Gento_Luna/Gento_Luna.xml')


class HeadObjectGraspJointTest(unittest.TestCase):
    def setUp(self):
        self.model = pin.buildModelFromMJCF(MODEL)
        self.data = self.model.createData()
        self.q = (np.asarray(self.model.lowerPositionLimit) +
                  np.asarray(self.model.upperPositionLimit)) / 2

    def pose(self, q, side):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        frame = self.model.getFrameId(f'{side}_tool')
        placement = self.data.oMf[frame]
        return (tuple(placement.translation),
                tuple(pin.Quaternion(placement.rotation).coeffs()))

    def fake_node(self, side):
        node = SimpleNamespace(
            ik_model=self.model, ik_data=self.data,
            ik_frame_id=self.model.getFrameId(f'{side}_tool'),
            index=0 if side == 'left' else 1,
            args=SimpleNamespace(position_only=False), phase='approach',
            target=None,
            user_joint_hold=self.q.tolist(),
            feedback_configuration=lambda: self.q.tolist())
        node.solve_arm_ik = lambda q, position, quaternion: HeadObjectGrasp.solve_arm_ik(
            node, q, position, quaternion)
        return node

    def test_zero_eef_delta_keeps_all_joints_with_tf_offset(self):
        for side in ('left', 'right'):
            with self.subTest(side=side):
                node = self.fake_node(side)
                position, quaternion = self.pose(self.q, side)
                actual = tuple(value + 0.025 for value in position)
                node.target = actual
                node.target_quaternion = quaternion
                result = HeadObjectGrasp.make_user_joint_target(node, actual, quaternion)
                np.testing.assert_allclose(result, self.q, atol=1e-8)

    def test_ik_changes_selected_arm_only(self):
        for side in ('left', 'right'):
            with self.subTest(side=side):
                node = self.fake_node(side)
                target_q = self.q.copy()
                start = 8 if side == 'left' else 15
                target_q[start + 1] += 0.01
                position, quaternion = self.pose(target_q, side)
                result = HeadObjectGrasp.solve_arm_ik(node, self.q.tolist(), position, quaternion)
                np.testing.assert_allclose(result[:start], self.q[:start], atol=1e-12)
                np.testing.assert_allclose(result[start + 7:], self.q[start + 7:], atol=1e-12)
                solved_position, _ = self.pose(np.asarray(result), side)
                self.assertLess(math.dist(solved_position, position), 0.003)

    def test_position_only_ik_reaches_position_without_moving_other_joints(self):
        for side in ('left', 'right'):
            with self.subTest(side=side):
                node = self.fake_node(side)
                node.args = SimpleNamespace(position_only=True,
                                            approach_angle_tolerance_deg=5,
                                            approach_position_tolerance_mm=10)
                target_q = self.q.copy()
                start = 8 if side == 'left' else 15
                target_q[start + 1] += 0.02
                position, _ = self.pose(target_q, side)
                _, held_quaternion = self.pose(self.q, side)
                node.target = position
                result = HeadObjectGrasp.solve_arm_ik(
                    node, self.q.tolist(), position, held_quaternion)
                np.testing.assert_allclose(result[:start], self.q[:start], atol=1e-12)
                np.testing.assert_allclose(result[start + 7:], self.q[start + 7:], atol=1e-12)
                solved_position, _ = self.pose(np.asarray(result), side)
                self.assertLess(math.dist(solved_position, position), 0.003)

    def test_position_only_ik_reports_separate_errors_when_unreachable(self):
        node = self.fake_node('left')
        node.args = SimpleNamespace(position_only=True,
                                    approach_angle_tolerance_deg=5,
                                    approach_position_tolerance_mm=10)
        node.target = (10.0, 10.0, 10.0)
        _, quaternion = self.pose(self.q, 'left')
        with self.assertRaisesRegex(ValueError, '位置误差=.*mm.*转角=.*°'):
            HeadObjectGrasp.solve_arm_ik(
                node, self.q.tolist(), node.target, quaternion)

    def test_user_joint_publishes_all_groups(self):
        publishers = [SimpleNamespace(publish=Mock()) for _ in range(4)]
        node = SimpleNamespace(user_joint_publishers=publishers,
                               get_clock=lambda: SimpleNamespace(
                                   now=lambda: SimpleNamespace(to_msg=lambda: Time())))
        q = [float(index) for index in range(22)]
        HeadObjectGrasp.publish_user_joint(node, q)
        messages = [publisher.publish.call_args.args[0] for publisher in publishers]
        self.assertEqual(list(messages[0].positions), q[8:15])
        self.assertEqual(list(messages[1].positions), q[15:22])
        self.assertEqual(list(messages[2].positions), q[:6])
        self.assertEqual(list(messages[3].positions), [q[6], q[7], 0.0])

    def test_source_switch_prefills_current_feedback(self):
        future = Mock()
        future.done.return_value = False
        client = SimpleNamespace(call_async=Mock(return_value=future))
        publish = Mock()
        node = SimpleNamespace(
            user_joint_hold=None, user_joint_target=None, disarm_started=None,
            joint_source=0, joint_switch_future=None, joint_switch_started=None,
            joint_select_client=client,
            feedback_configuration=lambda: self.q.tolist(),
            publish_user_joint=publish)
        self.assertFalse(HeadObjectGrasp.activate_user_joint(node, 10.0))
        self.assertFalse(HeadObjectGrasp.activate_user_joint(node, 10.25))
        client.call_async.assert_not_called()
        self.assertFalse(HeadObjectGrasp.activate_user_joint(node, 10.5))
        self.assertEqual(publish.call_count, 3)
        self.assertEqual(publish.call_args.args[0], self.q.tolist())
        self.assertEqual(client.call_async.call_args.args[0].data, 2)
        node.joint_source = 2
        self.assertTrue(HeadObjectGrasp.activate_user_joint(node, 10.55))

    def test_selected_arm_command_is_rate_limited(self):
        publish = Mock()
        current = self.q.tolist()
        desired = current.copy()
        desired[8] += 0.2
        node = SimpleNamespace(
            http_mode=False, joint_mode=True, joint_source=2, index=0,
            args=SimpleNamespace(max_joint_speed_rad_s=0.3),
            user_joint_target=current, make_user_joint_target=lambda *_: desired,
            publish_user_joint=publish)
        HeadObjectGrasp.publish_request(node, True, actual=(0, 0, 0),
                                        quaternion=(0, 0, 0, 1))
        command = publish.call_args.args[0]
        self.assertAlmostEqual(command[8] - current[8], 0.015)
        self.assertEqual(command[:8], current[:8])
        self.assertEqual(command[15:], current[15:])
        node.joint_source = 0
        with self.assertRaises(ValueError):
            HeadObjectGrasp.publish_request(node, True)
        self.assertEqual(publish.call_count, 1)

    def test_vlahost_publisher_does_not_block_preflight(self):
        now = time.monotonic()
        connected = lambda: SimpleNamespace(get_subscription_count=lambda: 1,
                                            topic_name='/tj/control/user/joint_cmd_A')
        node = SimpleNamespace(
            http_mode=False, joint_mode=True, joint_source=0, phase='preflight',
            joint_feedback_at=now, joint_positions=self.q.tolist(),
            joint_select_client=SimpleNamespace(service_is_ready=lambda: True),
            user_joint_publishers=[connected() for _ in range(4)],
            states=(2, 2, 1, 2), state_at=now, index=0,
            gripper_publisher=connected(), gripper_topic='/tj/control/gripperValueL',
            gripper_at=now, gripper_status_at=now, gripper_status=(1,),
            args=SimpleNamespace(feedback_timeout=0.5), label='左',
            get_publishers_info_by_topic=lambda _: [SimpleNamespace(node_name='luna_vlahost')])
        self.assertIsNone(HeadObjectGrasp.readiness_problem(node, now))

    def test_single_grasp_point_has_no_approach_offset(self):
        logger = SimpleNamespace(info=Mock())
        node = SimpleNamespace(
            object_pose=((0.3, 0.2, 0.8), (0., 0., 0., 1.)),
            args=SimpleNamespace(
                grasp_offset_mm=(0, 0, -40), approach_front_mm=0,
                approach_height_mm=0, lift_height_mm=1000,
                approach_only=True, position_only=True,
                max_target_angle_deg=180, max_displacement_mm=500),
            tool='left_tool', get_logger=lambda: logger)
        HeadObjectGrasp.make_goal(node, (0.1, 0.2, 0.7), (0., 0., 0., 1.))
        self.assertEqual(node.grasp_position, (0.3, 0.2, 0.76))
        self.assertEqual(node.approach_position, node.grasp_position)
        self.assertIn('唯一抓取点', logger.info.call_args.args[0])

    def test_single_grasp_point_finishes_without_second_motion(self):
        now = time.monotonic()
        finish = Mock()
        enter = Mock()
        point = (0.3, 0.2, 0.76)
        node = SimpleNamespace(
            exit_code=None, phase='approach', phase_started=now - 2,
            motion_started=now - 2, target=point,
            approach_position=point, grasp_position=point,
            target_quaternion=(0., 0., 0., 1.), settled_since=now - 2,
            last_log=now, label='左', finish=finish, enter=enter,
            args=SimpleNamespace(
                reach_only=False, approach_only=True, motion_timeout=15,
                approach_position_tolerance_mm=10,
                approach_angle_tolerance_deg=5, settle_time=1),
            actual=lambda: (point, (0., 0., 0., 1.)),
            readiness_problem=lambda _now: None,
            publish_request=Mock(), publish_gripper=Mock())
        HeadObjectGrasp.tick(node)
        self.assertEqual(finish.call_args.args[0], 0)
        self.assertIn('到达抓取点', finish.call_args.args[1])
        enter.assert_not_called()

    def test_intermediate_point_transitions_to_second_point(self):
        now = time.monotonic()
        finish = Mock()
        enter = Mock()
        first = (0.1, 0.2, 0.7)
        second = (0.2, 0.2, 0.7)
        node = SimpleNamespace(
            exit_code=None, phase='approach', phase_started=now - 2,
            motion_started=now - 2, target=first,
            intermediate_position=first, intermediate_reached=False,
            approach_position=second, grasp_position=second,
            target_quaternion=(0., 0., 0., 1.), settled_since=now - 2,
            last_log=now, label='左', finish=finish, enter=enter,
            args=SimpleNamespace(
                reach_only=False, approach_only=True, motion_timeout=15,
                approach_position_tolerance_mm=10,
                approach_angle_tolerance_deg=5, settle_time=1),
            actual=lambda: (first, (0., 0., 0., 1.)),
            readiness_problem=lambda _now: None,
            publish_request=Mock(), publish_gripper=Mock())
        HeadObjectGrasp.tick(node)
        enter.assert_called_once_with('approach_second', second)
        self.assertTrue(node.intermediate_reached)
        finish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
