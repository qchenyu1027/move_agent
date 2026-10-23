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
from tools.grasp_tool import GraspTool
from tools.move_tool import MoveTool, base_move_world, base_move_world_pose_list


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
            'base_move_world_pose_list.py', 'apriltag_left_tool_grasp_test.py',
            'base_move_world.py'])
        self.assertTrue(all(Path(command[2]).parent == SCRIPTS_DIR for command in commands))
        self.assertIn('--execute', commands[1])
        self.assertIn('--tag-id=5', commands[1])
        self.assertEqual(commands[0][3], '[1, 2]')

    def test_grasp_and_lift_mode_is_forwarded(self):
        command = GraspTool().build_command({
            'tag_id': 5, 'grasp_and_lift': True,
        })
        self.assertIn('--grasp-and-lift', command)

    def test_grasp_and_lift_must_be_boolean(self):
        with self.assertRaises(ValueError):
            GraspTool().build_command({'grasp_and_lift': 'true'})

    def test_failure_prevents_later_tasks(self):
        with patch('agent_tools.scheduler.run_command', side_effect=[3, 0]) as run, \
                self.output, contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(self.scheduler.run(self.tasks, execute=True), 3)
        self.assertEqual(run.call_count, 1)

    def test_invalid_later_task_prevents_all_execution(self):
        self.tasks.append({'tool': 'move', 'pose_ids': [4]})
        with patch('agent_tools.scheduler.run_command') as run:
            with self.assertRaisesRegex(ValueError, '任务 4'):
                self.scheduler.run(self.tasks, execute=True)
        run.assert_not_called()

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
            {'tool': 'grasp', 'tag_id': False},
            {'tool': 'grasp', 'tag_id': -1},
            {'tool': 'grasp', 'typo': 1},
            {'tool': 'unknown'},
            {'tool': []},
        ]
        for task in invalid:
            with self.subTest(task=task), self.assertRaises(ValueError):
                self.scheduler.prepare([task])

    def test_fixed_move_uses_script_default_target(self):
        command = MoveTool().build_command({'tool': 'move', 'fixed': True})
        self.assertEqual(Path(command[2]).name, 'base_move_world.py')
        self.assertEqual(len(command), 3)

    def test_rejects_options_that_change_task_semantics(self):
        for tool, task in [(MoveTool(), {'pose_ids': [1]}), (GraspTool(), {})]:
            for option in ['execute', 'exe', 'help', 'check-only', 'target-x-m']:
                with self.subTest(option=option), self.assertRaises(ValueError):
                    tool.build_command(dict(task, options={option: 1}))

    def test_option_value_cannot_become_command_flag(self):
        command = GraspTool().build_command({'options': {'offset-config': '--help'}})
        self.assertIn('--offset-config=--help', command)
        self.assertNotIn('--help', command)

    def test_scripts_and_data_live_in_tools_scripts(self):
        for name in ['base_move_world.py', 'base_move_world_pose_list.py',
                     'apriltag_left_tool_grasp_test.py', 'tag5_left_tool_offset.yaml',
                     'left_tool_joint_snapshot_20261010.yaml']:
            with self.subTest(name=name):
                self.assertTrue((SCRIPTS_DIR / name).is_file())

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
        for module in (base_move_world, base_move_world_pose_list,
                       base_move_world_pose_list.base_move_world):
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
