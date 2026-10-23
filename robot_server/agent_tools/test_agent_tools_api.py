# 用途：离线验证 HTTP 鉴权、任务执行结果和并发拒绝；运行：python3 -B -m unittest -v agent_tools.test_agent_tools_api。
"""仅模拟调度器，不连接机器人或监听网络端口。"""

import asyncio
import threading
import unittest
from unittest.mock import patch

import httpx

from agent_tools.api import create_app


class AgentToolsApiTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.health_patch = patch('agent_tools.api.inspect_robot_health', return_value={
            'ok': True, 'ready_for_motion': True, 'errors': [], 'warnings': [],
        })
        self.health_patch.start()
        self.client = httpx.AsyncClient(
            transport=httpx.ASGITransport(app=create_app('test-secret')),
            base_url='http://robot.test')
        self.headers = {'X-API-Key': 'test-secret'}

    async def asyncTearDown(self):
        await self.client.aclose()
        self.health_patch.stop()

    async def test_health_and_authentication(self):
        health = {'ok': True, 'ready_for_motion': True}
        with patch('agent_tools.api.inspect_robot_health', return_value=health):
            self.assertEqual((await self.client.get('/health')).json(), health)
        self.assertEqual((await self.client.post('/move')).status_code, 401)
        response = await self.client.post('/move', headers={'X-API-Key': 'bad'})
        self.assertEqual(response.status_code, 401)

    async def test_default_move_executes_fixed_target(self):
        with patch('agent_tools.api.TaskScheduler.run', return_value=0) as run:
            response = await self.client.post('/move', headers=self.headers)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), {'ok': True, 'message': '已到达目标位置'})
        run.assert_called_once_with([{'tool': 'move', 'fixed': True}], execute=True)

    async def test_move_cannot_override_tool(self):
        with patch('agent_tools.api.TaskScheduler.run', return_value=0) as run:
            response = await self.client.post(
                '/move', headers=self.headers, json={'tool': 'grasp', 'pose_ids': [1]})
        self.assertEqual(response.status_code, 200)
        run.assert_called_once_with([{'tool': 'move', 'pose_ids': [1]}], execute=True)

    async def test_invalid_move_does_not_execute(self):
        with patch('agent_tools.api.TaskScheduler.run') as run:
            response = await self.client.post(
                '/move', headers=self.headers, json={'pose_ids': [4]})
        self.assertEqual(response.status_code, 422)
        run.assert_not_called()

    async def test_unhealthy_robot_rejects_motion(self):
        health = {
            'ok': False, 'ready_for_motion': False,
            'errors': [], 'warnings': ['duplicate controller'],
        }
        with patch('agent_tools.api.inspect_robot_health', return_value=health), \
                patch('agent_tools.api.TaskScheduler.run') as run:
            response = await self.client.post('/move', headers=self.headers)
        self.assertEqual(response.status_code, 503)
        self.assertEqual(response.json()['health'], health)
        run.assert_not_called()

    async def test_grasp_failure_reports_exit_code(self):
        with patch('agent_tools.api.TaskScheduler.run', return_value=3):
            response = await self.client.post('/grasp', headers=self.headers,
                                              json={'tag_id': 5})
        self.assertEqual(response.status_code, 500)
        self.assertEqual(response.json()['exit_code'], 3)
        self.assertFalse(response.json()['ok'])

    async def test_plan_validation_prevents_any_action(self):
        tasks = [{'tool': 'move', 'fixed': True}, {'tool': 'move', 'pose_ids': [4]}]
        with patch('agent_tools.api.TaskScheduler.run') as run:
            response = await self.client.post('/tasks', headers=self.headers, json=tasks)
        self.assertEqual(response.status_code, 422)
        run.assert_not_called()

    async def test_missing_api_key_rejects_startup(self):
        app = create_app('')
        with self.assertRaisesRegex(RuntimeError, 'ROBOT_API_KEY'):
            async with app.router.lifespan_context(app):
                pass

    async def test_concurrent_action_is_rejected(self):
        started = threading.Event()
        release = threading.Event()
        results = []

        def wait_for_release(_tasks, *, execute):
            self.assertTrue(execute)
            started.set()
            self.assertTrue(release.wait(5))
            return 0

        with patch('agent_tools.api.TaskScheduler.run', side_effect=wait_for_release):
            worker = asyncio.create_task(self.client.post('/move', headers=self.headers))
            try:
                async def wait_for_start():
                    while not started.is_set():
                        await asyncio.sleep(0.01)

                await asyncio.wait_for(wait_for_start(), 3)
                self.assertTrue(started.is_set())
                response = await self.client.post('/move', headers=self.headers)
                self.assertEqual(response.status_code, 409)
                self.assertTrue(response.json()['retryable'])
            finally:
                release.set()
                results.append(await asyncio.wait_for(worker, 4))
        self.assertEqual(results[0].status_code, 200)

    async def test_cancel_requests_safe_child_stop(self):
        started = threading.Event()
        release = threading.Event()

        def wait_for_release(_tasks, *, execute):
            self.assertTrue(execute)
            started.set()
            self.assertTrue(release.wait(5))
            return 130

        with patch('agent_tools.api.TaskScheduler.run', side_effect=wait_for_release):
            worker = asyncio.create_task(self.client.post('/move', headers=self.headers))
            while not started.is_set():
                await asyncio.sleep(0.01)
            response = await self.client.post('/cancel', headers=self.headers)
            self.assertEqual(response.status_code, 202)
            release.set()
            result = await asyncio.wait_for(worker, 4)
        self.assertEqual(result.status_code, 409)
        self.assertTrue(result.json()['cancelled'])

    async def test_cancel_rejects_when_idle(self):
        response = await self.client.post('/cancel', headers=self.headers)
        self.assertEqual(response.status_code, 409)
        self.assertFalse(response.json()['cancelled'])

    async def test_client_cancellation_keeps_action_lock(self):
        started = threading.Event()
        release = threading.Event()
        completed = threading.Event()

        def wait_for_release(_tasks, *, execute):
            started.set()
            self.assertTrue(release.wait(5))
            completed.set()
            return 0

        with patch('agent_tools.api.TaskScheduler.run', side_effect=wait_for_release):
            worker = asyncio.create_task(self.client.post('/move', headers=self.headers))
            try:
                async def wait_for_start():
                    while not started.is_set():
                        await asyncio.sleep(0.01)

                await asyncio.wait_for(wait_for_start(), 3)
                worker.cancel()
                with self.assertRaises(asyncio.CancelledError):
                    await worker
                response = await self.client.post('/move', headers=self.headers)
                self.assertEqual(response.status_code, 409)
            finally:
                release.set()
                async def wait_for_completion():
                    while not completed.is_set():
                        await asyncio.sleep(0.01)

                await asyncio.wait_for(wait_for_completion(), 3)


if __name__ == '__main__':
    unittest.main()
