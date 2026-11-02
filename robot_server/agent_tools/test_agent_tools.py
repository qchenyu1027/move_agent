# 用途：离线验证调度顺序、失败停止和工具参数，不连接机器人；运行：python3 -B -m unittest -v agent_tools.test_agent_tools。
"""仅使用模拟控制进程验证调度行为。"""

import ast
import contextlib
import io
from pathlib import Path
import signal
import subprocess
import threading
import unittest
from unittest.mock import Mock, patch

from agent_tools.scheduler import TaskScheduler
from tools.common import SCRIPTS_DIR, run_command
from tools.arm_teleop_tool import ArmTeleopTool
from tools.control_sources_tool import ControlSourcesTool
from tools.go_home_tool import GoHomeTool
from tools.scripts.go_home import started_duration
from tools.scripts import select_control_sources
from tools.gripper_tool import GripperTool
from tools.grasp_tool import GraspTool
from tools.head_object_grasp_tool import HeadObjectGraspTool
from tools.move_tool import MoveTool, native_nav_targets
from tools.move_xy_tool import MoveXYTool
from tools.torso_tool import TorsoTool
from tools.scripts.torso_home_down import (
    CONTROLLED_INDICES, _command_positions, _forward_kinematics, _load_model,
    _torso_residual, solve_height_offset,
)


class AgentToolsTest(unittest.TestCase):
    def setUp(self):
        self.scheduler = TaskScheduler()
        self.tasks = [
            {'tool': 'move', 'pose_ids': [1, 2]},
            {'tool': 'grasp', 'tag_id': 5},
            {'tool': 'move', 'pose': [0.1, 0.2, 0.0, 0, 0, 0, 1]},
        ]
        self.output = contextlib.redirect_stdout(io.StringIO())

    def test_preview_never_launches_process(self):
        with patch('agent_tools.scheduler.run_command') as run, self.output:
            self.assertEqual(self.scheduler.run(self.tasks), 0)
        run.assert_not_called()

    def test_order_and_grasp_execution_flag(self):
        with patch('agent_tools.scheduler.run_command', return_value=0) as run, self.output:
            self.assertEqual(self.scheduler.run(self.tasks, execute=True), 0)
        commands = [call.args[0] for call in run.call_args_list]
        self.assertEqual([Path(command[2]).name for command in commands], [
            'native_nav_test.py', 'apriltag_left_tool_grasp_test.py',
            'native_nav_test.py'])
        self.assertTrue(all(Path(command[2]).parent == SCRIPTS_DIR for command in commands))
        self.assertIn('--execute', commands[1])
        self.assertIn('--tag-id=5', commands[1])
        self.assertEqual(commands[0][3:5], ['--pose-ids', '[1, 2]'])
        self.assertTrue('--wait' in commands[0] and '--execute' in commands[0])

    def test_grasp_and_lift_mode_is_forwarded(self):
        command = GraspTool().build_command({
            'tag_id': 5, 'grasp_and_lift': True,
        })
        self.assertIn('--grasp-and-lift', command)

    def test_grasp_and_lift_must_be_boolean(self):
        with self.assertRaises(ValueError):
            GraspTool().build_command({'grasp_and_lift': 'true'})

    def test_head_object_grasp_builds_offset_command(self):
        command = HeadObjectGraspTool().build_command({
            'side': 'left',
            'grasp_offset_mm': [0, 0, -40],
            'options': {'position-only': True, 'reach-only': True, 'ik-source': 2,
                        'joint-source': 0},
        })
        self.assertEqual(Path(command[2]).name, 'head_object_grasp.py')
        self.assertIn('--object-pose-topic=/object_pose', command)
        self.assertIn('--pose-request-topic=/f2g_pose/request', command)
        self.assertIn('--grasp-offset-mm', command)
        self.assertIn('--reach-only', command)
        start = command.index('--grasp-offset-mm')
        self.assertEqual(command[start + 1:start + 4], ['0', '0', '-40'])
        self.assertIn('--position-only', command)
        self.assertIn('--ik-source=2', command)

    def test_head_object_grasp_validates_offset(self):
        with self.assertRaises(ValueError):
            HeadObjectGraspTool().build_command({
                'side': 'left', 'grasp_offset_mm': [0, 0]})

    def test_control_sources_precedes_bottle_grasp(self):
        import json
        plan = json.loads((Path(__file__).parent / 'bottle_transport_2_to_3.json')
                          .read_text(encoding='utf-8'))
        commands = self.scheduler.prepare(plan)
        grasp_index = next(i for i, task in enumerate(plan)
                           if task['tool'] == 'head_object_grasp')
        self.assertEqual(plan[grasp_index - 1], {
            'tool': 'control_sources', 'ik_source': 2, 'joint_source': 0})
        self.assertEqual(Path(commands[grasp_index - 1][2]).name,
                         'select_control_sources.py')
        self.assertIn('--ik-source=2', commands[grasp_index - 1])
        self.assertIn('--joint-source=0', commands[grasp_index - 1])

    def test_control_sources_rejects_invalid_source(self):
        for task in (
                {'ik_source': True, 'joint_source': 0},
                {'ik_source': 2, 'joint_source': -1},
                {'ik_source': 2},
                {'ik_source': 2, 'joint_source': 0, 'options': {'execute': True}}):
            with self.subTest(task=task), self.assertRaises(ValueError):
                ControlSourcesTool().build_command(task)

    def test_control_sources_stops_on_unexpected_active_source(self):
        node = Mock()
        client = node.create_client.return_value
        client.wait_for_service.return_value = True
        future = client.call_async.return_value
        future.done.return_value = True
        future.result.return_value = Mock(success=True, error_code=1, message='active=1')
        with (patch.object(select_control_sources, 'Int') as service_type,
              patch.object(select_control_sources, 'rclpy') as ros):
            with self.assertRaisesRegex(RuntimeError, '实际源=1'):
                select_control_sources.select_source(
                    node, '/tj/control/ik_request_mux/select', 2, 1.0)
            self.assertEqual(service_type.Request.return_value.data, 2)
            ros.spin_until_future_complete.assert_called_once_with(
                node, future, timeout_sec=1.0)

    def test_move_xy_builds_relative_offset_command(self):
        command = MoveXYTool().build_command({'x_m': 0.2, 'y_m': -0.1})
        self.assertEqual(Path(command[2]).name, 'native_nav_test.py')
        self.assertIn('--offset-x-m=0.2', command)
        self.assertIn('--offset-y-m=-0.1', command)
        aliases = MoveXYTool().build_command({'x': 0.2, 'y': -0.1})
        self.assertIn('--offset-x-m=0.2', aliases)
        self.assertIn('--offset-y-m=-0.1', aliases)

    def test_torso_home_down_builds_signed_offset_command(self):
        command = TorsoTool().build_command({'offset_mm': -10})
        self.assertEqual(Path(command[2]).name, 'torso_home_down.py')
        self.assertIn('--offset-mm=-10', command)
        self.assertIn('--offset-mm=10', TorsoTool().build_command({'offset_mm': 10}))
        self.assertIn('--joint-log-hz=2', TorsoTool().build_command({
            'offset_mm': -10, 'options': {'joint-log-hz': 2}}))

    def test_go_home_builds_ros_command(self):
        command = GoHomeTool().build_command({'tool': 'go_home'})
        self.assertEqual(Path(command[2]).name, 'go_home.py')
        self.assertNotIn('--execute', command)
        command = GoHomeTool().build_command({
            'options': {'namespace': 'robot', 'wait-timeout-s': 10}})
        self.assertIn('--namespace=robot', command)
        self.assertIn('--wait-timeout-s=10', command)

    def test_go_home_rejects_motion_options(self):
        for task in ({'offset_mm': -10}, {'options': {'execute': True}}):
            with self.subTest(task=task), self.assertRaises(ValueError):
                GoHomeTool().build_command(task)

    def test_go_home_waits_for_reported_duration(self):
        self.assertEqual(started_duration(
            'Go home started, duration=4.250000s, nq=22'), 4.25)
        with self.assertRaises(ValueError):
            started_duration('Go home completed')

    def test_arm_teleop_builds_left_and_right_ros_commands(self):
        for side in ('left', 'right'):
            with self.subTest(side=side):
                command = ArmTeleopTool().build_command({
                    'tool': 'arm_teleop', 'side': side,
                    'options': {'incremental': True, 'position-step-mm': 3},
                })
                self.assertEqual(Path(command[2]).name, 'arm_teleop.py')
                self.assertIn(f'--side={side}', command)
                self.assertIn('--execute', command)
                self.assertIn('--incremental', command)
                self.assertIn('--position-step-mm=3', command)
                self.assertNotIn('--control-backend=vlahost', command)
                vla_command = ArmTeleopTool().build_command({
                    'tool': 'arm_teleop', 'side': side,
                    'options': {
                        'control-backend': 'vlahost',
                        'vlahost-url': 'http://6.6.7.100:8011',
                    },
                })
                self.assertIn('--control-backend=vlahost', vla_command)
                self.assertIn('--vlahost-url=http://6.6.7.100:8011', vla_command)

    def test_arm_teleop_builds_one_shot_translation_command(self):
        command = ArmTeleopTool().build_command({
            'side': 'left',
            'options': {'move-z-mm': 50, 'once-timeout-s': 15},
        })
        self.assertIn('--move-z-mm=50', command)
        self.assertIn('--once-timeout-s=15', command)

    def test_arm_teleop_validates_side_and_options(self):
        for task in (
                {'side': 'middle'},
                {'side': 'left', 'options': {'execute': True}},
                {'side': 'right', 'pose': [0, 0, 0]}):
            with self.subTest(task=task), self.assertRaises(ValueError):
                ArmTeleopTool().build_command(task)

    def test_gripper_builds_open_close_and_position_commands(self):
        command = GripperTool().build_command({
            'tool': 'gripper', 'side': 'left', 'action': 'open',
            'position': 0.7, 'options': {'speed': 0.4},
        })
        self.assertEqual(Path(command[2]).name, 'gripper_control.py')
        self.assertIn('--side=left', command)
        self.assertIn('--target-position=0.7', command)
        self.assertIn('--speed=0.4', command)
        self.assertIn('--control-backend=ros', GripperTool().build_command({
            'side': 'left', 'action': 'open',
            'options': {'control-backend': 'ros'}}))
        self.assertIn('--control-backend=vlahost', GripperTool().build_command({
            'side': 'left', 'action': 'open',
            'options': {'control-backend': 'vlahost',
                        'vlahost-url': 'http://6.6.7.100:8011'}}))
        self.assertIn('--close', GripperTool().build_command({
            'side': 'right', 'action': 'close'}))

    def test_gripper_validates_target(self):
        for task in (
                {'side': 'left', 'action': 'set'},
                {'side': 'middle', 'action': 'open'},
                {'side': 'left', 'action': 'set', 'position': 1.1},
                {'side': 'left', 'action': 'open', 'options': {'unknown': 1}}):
            with self.subTest(task=task), self.assertRaises(ValueError):
                GripperTool().build_command(task)

    def test_torso_home_down_supports_legacy_down_mm(self):
        command = TorsoTool().build_command({'down_mm': 10})
        self.assertIn('--offset-mm=-10', command)

    def test_torso_home_down_rejects_unsafe_values(self):
        for value in (0, float('nan'), float('inf'), True, '10'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                TorsoTool().build_command({'offset_mm': value})

    def test_torso_home_down_allows_offset_beyond_50mm(self):
        # 已去掉 50mm 人为上限；幅度改由脚本的 IK 收敛和关节限位约束。
        self.assertIn('--offset-mm=80', TorsoTool().build_command({'offset_mm': 80}))
        self.assertIn('--offset-mm=-120', TorsoTool().build_command({'offset_mm': -120}))

    def test_torso_height_solver_moves_all_body_joints_and_hits_target(self):
        home = [
            0.0, 0.4, -0.6, 0.2, 0.0, 0.0, -0.5235958, 0.0,
            1.557287, -0.987363, -1.329712, -1.985228, -0.123364, 0.609076, -0.231212,
            -1.767831, -1.444084, 1.359199, -1.828346, 0.012112, -0.081909, -0.106920,
        ]
        lowered, before, after = solve_height_offset(
            '/home/liuqian/WBC/Apex_Deploy/robot_node/marvin_description/'
            'mjcf/Gento_Luna/Gento_Luna.xml', home, -10.0)
        self.assertAlmostEqual((after - before) * 1000.0, -10.0, places=3)
        self.assertTrue(any(abs(lowered[i] - home[i]) > 1e-4 for i in range(6)))
        import numpy as np
        self.assertTrue(np.array_equal(lowered[8:], np.asarray(home[8:])))

    def test_torso_height_solver_uses_real_leg_j4_upper_limit(self):
        model = '/home/liuqian/WBC/Apex_Deploy/robot_node/marvin_description/' \
                'mjcf/Gento_Luna/Gento_Luna.xml'
        _, names, lower, upper = _load_model(model)
        j4 = names.index('Leg_J4_Joint')
        self.assertAlmostEqual(upper[j4], 0.8)
        self.assertAlmostEqual(lower[j4], -0.87)
        xml_root = __import__('xml.etree.ElementTree', fromlist=['ElementTree']).parse(model).getroot()
        xml_joints = xml_root.findall('./worldbody//joint')
        for index in range(len(names)):
            if index != j4:
                xml_upper = float(xml_joints[index].get('range').split()[1])
                self.assertAlmostEqual(upper[index], xml_upper)

        home = [
            0.0, 0.4, -0.6, 0.2, 0.0, 0.0, -0.5235958, 0.0,
            1.557287, -0.987363, -1.329712, -1.985228, -0.123364, 0.609076, -0.231212,
            -1.767831, -1.444084, 1.359199, -1.828346, 0.012112, -0.081909, -0.106920,
        ]
        lowered, before, after = solve_height_offset(model, home, -234.0)
        self.assertAlmostEqual((after - before) * 1000.0, -234.0, places=2)
        self.assertLessEqual(lowered[j4], 0.8 + 1e-9)
        import numpy as np
        self.assertTrue(np.array_equal(lowered[8:], np.asarray(home[8:])))

        # 放开 X 后，下降超过原先的水平·原地分支仍可求解；代价是明显前移。
        root, names, _, _ = _load_model(model)
        home_array = np.asarray(home)
        base = _forward_kinematics(root, names, home_array)['torso_link']
        lowered, before, after = solve_height_offset(model, home, -250.0)
        target_pose = _forward_kinematics(root, names, lowered)['torso_link']
        delta_xyz = target_pose[:3, 3] - base[:3, 3]
        self.assertAlmostEqual(delta_xyz[1], 0.0, places=6)
        self.assertAlmostEqual(delta_xyz[2] * 1000.0, -250.0, places=2)
        self.assertGreater(abs(delta_xyz[0]), 0.25)
        self.assertLess(np.max(np.abs(target_pose[:3, :3] - base[:3, :3])), 1e-6)
        self.assertTrue(np.array_equal(lowered[8:], home_array[8:]))

    def test_torso_height_solver_releases_front_back_position(self):
        """高度 IK 只保持 Y、Z 和姿态，目标 X 变化不应产生残差。"""
        model = '/home/liuqian/WBC/Apex_Deploy/robot_node/marvin_description/' \
                'mjcf/Gento_Luna/Gento_Luna.xml'
        root, names, _, _ = _load_model(model)
        home = [
            0.0, 0.4, -0.6, 0.2, 0.0, 0.0, -0.5235958, 0.0,
            1.557287, -0.987363, -1.329712, -1.985228, -0.123364, 0.609076, -0.231212,
            -1.767831, -1.444084, 1.359199, -1.828346, 0.012112, -0.081909, -0.106920,
        ]
        pose = _forward_kinematics(root, names, home)['torso_link']
        target = pose.copy()
        target[0, 3] += 0.5
        residual = _torso_residual(pose, target)
        self.assertEqual(residual.size, 5)
        import numpy as np
        self.assertTrue(np.allclose(residual, 0.0, atol=1e-12))

    def test_torso_command_positions_have_ros_message_lengths(self):
        import numpy as np

        self.assertEqual(CONTROLLED_INDICES, list(range(6)) + list(range(8, 22)))
        for q in (list(range(22)), np.arange(22, dtype=float)):
            with self.subTest(type=type(q)):
                left, right, body, head = _command_positions(q, [-0.4, 0.1])
                self.assertEqual([len(part) for part in (left, right, body, head)],
                                 [7, 7, 6, 3])
                self.assertEqual(head, [-0.4, 0.1, 0.0])
        with self.assertRaises(ValueError):
            _command_positions([0.0] * 21, [0.0, 0.0])
        with self.assertRaises(ValueError):
            _command_positions([float('nan')] + [0.0] * 21, [0.0, 0.0])
        with self.assertRaises(ValueError):
            _command_positions([0.0] * 22, [float('nan'), 0.0])

    def test_move_xy_requires_finite_x_and_y(self):
        for task in (
                {'x_m': 0.2},
                {'y_m': 0.1},
                {'x_m': True, 'y_m': 0.1},
                {'x_m': float('nan'), 'y_m': 0.1},
                {'x_m': 0.2, 'y_m': '0.1'},
                {'x': 0.2, 'x_m': 0.2, 'y': 0.1},
                {'x_m': 0.2, 'y_m': 0.1, 'z_m': 0.0}):
            with self.subTest(task=task), self.assertRaises(ValueError):
                MoveXYTool().build_command(task)

    def test_failure_prevents_later_tasks(self):
        with patch('agent_tools.scheduler.run_command', side_effect=[3, 0]) as run, \
                self.output, contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(self.scheduler.run(self.tasks, execute=True), 3)
        self.assertEqual(run.call_count, 1)

    def test_invalid_later_task_prevents_all_execution(self):
        self.tasks.append({'tool': 'move', 'pose_ids': [6]})
        with patch('agent_tools.scheduler.run_command') as run:
            with self.assertRaisesRegex(ValueError, '任务 4'):
                self.scheduler.run(self.tasks, execute=True)
        run.assert_not_called()

    def test_pose4_is_available_with_expected_world_target(self):
        command = MoveTool().build_command({'tool': 'move', 'pose_ids': [4]})
        self.assertEqual(command[3:5], ['--pose-ids', '[4]'])
        target = native_nav_targets.pose_to_target(
            native_nav_targets.DEFAULT_POSES[3], 4)
        self.assertAlmostEqual(target[0], -0.4597338938554738)
        self.assertAlmostEqual(target[1], -0.057127684192941855)
        self.assertAlmostEqual(target[2], 0.016258406584268782)
        self.assertAlmostEqual(target[3], -3.074536924110893)

    def test_invalid_targets_and_unknown_fields(self):
        invalid = [
            {'tool': 'move'},
            {'tool': 'move', 'fixed': False},
            {'tool': 'move', 'fixed': True, 'pose_ids': [1]},
            {'tool': 'move', 'pose_ids': [True]},
            {'tool': 'move', 'pose_ids': []},
            {'tool': 'move', 'pose_ids': '[1]'},
            {'tool': 'move', 'pose_ids': [1], 'pose': [0] * 7},
            {'tool': 'move', 'pose': [0] * 7},
            {'tool': 'move', 'pose': [float('nan')] * 7},
            {'tool': 'move_xy'},
            {'tool': 'move_xy', 'x_m': 0.1},
            {'tool': 'move_xy', 'x_m': 0.1, 'y_m': float('inf')},
            {'tool': 'grasp', 'tag_id': False},
            {'tool': 'grasp', 'tag_id': -1},
            {'tool': 'grasp', 'typo': 1},
            {'tool': 'unknown'},
            {'tool': 'torso_home_down'},
            {'tool': 'torso_home_down', 'down_mm': 0},
            {'tool': 'go_home', 'pose': [0, 0, 0]},
            {'tool': 'arm_teleop'},
            {'tool': []},
        ]
        for task in invalid:
            with self.subTest(task=task), self.assertRaises(ValueError):
                self.scheduler.prepare([task])

    def test_fixed_move_uses_script_default_target(self):
        command = MoveTool().build_command({'tool': 'move', 'fixed': True})
        self.assertEqual(Path(command[2]).name, 'native_nav_test.py')
        self.assertEqual(command[3:], ['--fixed', '--wait', '--execute'])

    def test_rejects_options_that_change_task_semantics(self):
        for tool, task in [
                (MoveTool(), {'pose_ids': [1]}),
                (MoveXYTool(), {'x_m': 0.1, 'y_m': 0.1}),
                (GraspTool(), {})]:
            for option in ['execute', 'exe', 'help', 'check-only', 'target-x-m']:
                with self.subTest(option=option), self.assertRaises(ValueError):
                    tool.build_command(dict(task, options={option: 1}))

    def test_old_navigation_options_are_rejected(self):
        for tool, task in (
                (MoveTool(), {'pose_ids': [4]}),
                (MoveXYTool(), {'x_m': 0.1, 'y_m': 0.0})):
            for option in ('action-name', 'navigation-type', 'odometry-retries'):
                with self.subTest(tool=type(tool).__name__, option=option):
                    with self.assertRaises(ValueError):
                        tool.build_command(dict(task, options={option: 1}))

    def test_option_value_cannot_become_command_flag(self):
        command = GraspTool().build_command({'options': {'offset-config': '--help'}})
        self.assertIn('--offset-config=--help', command)
        self.assertNotIn('--help', command)

    def test_scripts_and_data_live_in_tools_scripts(self):
        for name in ['native_nav_test.py', 'native_nav_targets.py', 'pos.yaml',
                     'apriltag_left_tool_grasp_test.py', 'tag5_left_tool_offset.yaml',
                     'left_tool_joint_snapshot_20261010.yaml', 'go_home.py',
                     'arm_teleop.py', 'right_arm_teleop.py', 'gripper_control.py']:
            with self.subTest(name=name):
                self.assertTrue((SCRIPTS_DIR / name).is_file())

    def test_arm_teleop_entry_uses_local_script(self):
        from tools.scripts.arm_teleop import ROOT_SCRIPT
        self.assertEqual(ROOT_SCRIPT, SCRIPTS_DIR / 'right_arm_teleop.py')

    def test_grasp_script_is_self_contained(self):
        """抓取脚本已合并为单文件，不能再导入 tools/scripts 下的同级脚本。"""
        source = (SCRIPTS_DIR / 'apriltag_left_tool_grasp_test.py').read_text(encoding='utf-8')
        imported = set()
        for node in ast.walk(ast.parse(source)):
            if isinstance(node, ast.Import):
                imported |= {alias.name.split('.')[0] for alias in node.names}
            elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
                imported.add(node.module.split('.')[0])
        siblings = {path.stem for path in SCRIPTS_DIR.glob('*.py')}
        self.assertEqual(imported & siblings, set())

    def test_move_tool_loads_scripts_copy_not_root_original(self):
        for module in (native_nav_targets,):
            with self.subTest(module=module.__name__):
                self.assertEqual(Path(module.__file__).parent, SCRIPTS_DIR)

    def test_process_uses_scripts_directory_without_shell(self):
        process = Mock()
        process.wait.return_value = 0
        with patch('tools.common.subprocess.Popen', return_value=process) as popen:
            self.assertEqual(run_command(['python3', 'script.py']), 0)
        popen.assert_called_once_with(['python3', 'script.py'], cwd=SCRIPTS_DIR)

    def test_interrupt_signals_child_and_waits_for_cleanup(self):
        process = Mock()
        process.wait.side_effect = [KeyboardInterrupt, 130]
        process.poll.return_value = None
        with patch('tools.common.subprocess.Popen', return_value=process):
            self.assertEqual(run_command(['python3', 'script.py']), 130)
        process.send_signal.assert_called_once_with(signal.SIGINT)
        self.assertEqual(process.wait.call_count, 2)

    def test_cancel_event_signals_child_and_waits_for_cleanup(self):
        process = Mock()
        process.wait.side_effect = [
            subprocess.TimeoutExpired(['python3'], 0.1), 130]
        process.poll.return_value = None
        cancel_event = threading.Event()
        cancel_event.set()
        with patch('tools.common.subprocess.Popen', return_value=process):
            self.assertEqual(
                run_command(['python3', 'script.py'], cancel_event), 130)
        process.send_signal.assert_called_once_with(signal.SIGINT)


if __name__ == '__main__':
    unittest.main()
