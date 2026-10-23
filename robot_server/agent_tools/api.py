# 用途：通过 FastAPI 对局域网提供 move、grasp 和串行任务接口；运行：先设置 ROBOT_API_KEY，再执行 python3 -B -m agent_tools.api --host 0.0.0.0。
"""将现有任务调度器封装为需要 API key 的 HTTP 服务。"""

import argparse
import asyncio
from collections import Counter
from contextlib import asynccontextmanager
import json
import logging
import os
from secrets import compare_digest
import subprocess
import threading
from urllib.request import ProxyHandler, Request, build_opener

from fastapi import Body, Depends, FastAPI, Header, HTTPException, Request as FastAPIRequest
from fastapi.responses import JSONResponse
import uvicorn

from agent_tools.scheduler import TaskScheduler


LOGGER = logging.getLogger(__name__)
VLAHOST_URL = os.environ.get('VLAHOST_URL', 'http://127.0.0.1:8011').rstrip('/')
REQUIRED_ROS_NODES = {'/luna_vlahost', '/base_teleop', '/base_controller'}
REQUIRED_ROS_SERVICES = {'/control/base_teleop/set_mode'}
REQUIRED_ROS_TOPICS = {'/odomAftUpdate', '/info/base_teleop/active_mode', '/tj/joint_states'}


def _http_json(path):
    opener = build_opener(ProxyHandler({}))
    with opener.open(Request(VLAHOST_URL + path), timeout=2.0) as response:
        return json.load(response)


def _ros_names(kind):
    result = subprocess.run(
        ['ros2', kind, 'list'], capture_output=True, text=True, timeout=3.0,
        check=False)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or f'ros2 {kind} list failed')
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def inspect_robot_health(busy):
    """汇总只读 HTTP/ROS 状态；任何检查失败都返回结构化原因。"""
    errors = []
    warnings = []
    try:
        vlahost_health = _http_json('/health')
        state = _http_json('/state')
        pose_response = _http_json('/base_world_pose')
    except Exception as error:
        errors.append(f'VLAHost HTTP 检查失败：{error}')
        vlahost_health, state, pose_response = {}, {}, {}

    if not vlahost_health.get('ok') or not vlahost_health.get('ros_feedback'):
        errors.append('VLAHost 健康检查或 ROS 反馈异常')

    pose = pose_response.get('pose') if isinstance(pose_response, dict) else None
    pose = pose if isinstance(pose, dict) else {}
    pose_age = pose.get('received_age_s')
    pose_fresh = (isinstance(pose_age, (int, float)) and
                  not isinstance(pose_age, bool) and 0.0 <= pose_age <= 0.5)
    if not state.get('connected'):
        errors.append('VLAHost 未连接机器人反馈')
    if not pose.get('valid') or not pose_fresh:
        errors.append('camera_init -> body 世界位姿无效或超过 0.5 s')

    ros = {}
    for kind, required in (
            ('node', REQUIRED_ROS_NODES), ('service', REQUIRED_ROS_SERVICES),
            ('topic', REQUIRED_ROS_TOPICS)):
        try:
            names = _ros_names(kind)
            counts = Counter(names)
            missing = sorted(required - set(names))
            duplicates = sorted(name for name in required if counts[name] > 1)
            ros[kind + 's'] = {
                'ok': not missing and not duplicates,
                'missing': missing,
                'duplicates': duplicates,
            }
            if missing:
                errors.append(f'ROS {kind} 缺失：{", ".join(missing)}')
            if duplicates:
                warnings.append(f'ROS {kind} 重名：{", ".join(duplicates)}')
        except Exception as error:
            ros[kind + 's'] = {'ok': False, 'error': str(error)}
            errors.append(f'ROS {kind} 检查失败：{error}')

    control_mode = state.get('control_mode') if isinstance(state, dict) else {}
    return {
        'ok': not errors and not warnings,
        'ready_for_motion': not errors and not warnings and not busy,
        'api': {'busy': busy},
        'vlahost': {
            'ok': bool(vlahost_health.get('ok')),
            'ros_feedback': bool(vlahost_health.get('ros_feedback')),
            'connected': bool(state.get('connected')),
        },
        'robot': {
            'base_world_pose': {
                'valid': bool(pose.get('valid')),
                'received_age_s': pose_age,
                'frame_id': pose.get('frame_id'),
                'child_frame_id': pose.get('child_frame_id'),
            },
            'base_teleop_mode': (control_mode or {}).get('base_teleop_mode'),
            'left_gripper': state.get('gripper_left'),
            'right_gripper': state.get('gripper_right'),
            'intervention': state.get('intervention'),
        },
        'ros': ros,
        'errors': errors,
        'warnings': warnings,
    }


def create_app(api_key=None):
    """创建单进程服务；所有会移动机器人的请求共用一把锁。"""
    key = api_key if api_key is not None else os.environ.get('ROBOT_API_KEY', '')

    @asynccontextmanager
    async def lifespan(_app):
        if not key:
            raise RuntimeError('必须设置 ROBOT_API_KEY 才能启动机器人 HTTP 服务')
        yield

    app = FastAPI(title='Robot Agent Tools API', lifespan=lifespan)
    action_lock = threading.Lock()
    scheduler = TaskScheduler()

    async def authorize(x_api_key: str | None = Header(default=None)):
        if not key or x_api_key is None or not compare_digest(x_api_key, key):
            raise HTTPException(status_code=401, detail='API key 无效')

    async def execute(tasks, success_message, request):
        try:
            scheduler.prepare(tasks)
        except ValueError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error

        health = await asyncio.to_thread(inspect_robot_health, False)
        if not health['ready_for_motion']:
            return JSONResponse(status_code=503, content={
                'ok': False,
                'message': '机器人健康检查未通过，未启动任务',
                'retryable': True,
                'health': health,
            })

        if not action_lock.acquire(blocking=False):
            return JSONResponse(status_code=409, content={
                'ok': False, 'message': '机器人正在执行其他任务', 'retryable': True,
            })
        scheduler.reset_cancel()
        # 动作结束时才释放锁，即使 HTTP 客户端提前断开也不允许启动下一项。
        loop = asyncio.get_running_loop()
        outcome = loop.create_future()

        def run_action():
            try:
                code = scheduler.run(tasks, execute=True)
            except BaseException as error:
                failure = error
                code = None
            else:
                failure = None
            scheduler.reset_cancel()
            action_lock.release()

            def publish_result():
                if failure is None:
                    outcome.set_result(code)
                else:
                    outcome.set_exception(failure)

            try:
                loop.call_soon_threadsafe(publish_result)
            except RuntimeError:
                LOGGER.exception('HTTP 服务已停止，机器人任务结果无法发送给客户端')

        try:
            threading.Thread(target=run_action, name='robot-action').start()
        except BaseException:
            action_lock.release()
            raise
        try:
            while not outcome.done():
                try:
                    code = await asyncio.wait_for(asyncio.shield(outcome), 0.2)
                    break
                except asyncio.TimeoutError:
                    if await request.is_disconnected():
                        scheduler.cancel_current()
            else:
                code = outcome.result()
        except asyncio.CancelledError:
            scheduler.cancel_current()
            raise
        except OSError:
            LOGGER.exception('控制脚本启动失败')
            return JSONResponse(status_code=500, content={
                'ok': False, 'message': '控制脚本启动失败', 'retryable': False,
            })
        if code == 130:
            return JSONResponse(status_code=409, content={
                'ok': False, 'message': '机器人任务已取消',
                'retryable': False, 'cancelled': True, 'exit_code': code,
            })
        if code != 0:
            return JSONResponse(status_code=500, content={
                'ok': False, 'message': f'机器人任务失败，退出码 {code}',
                'retryable': False, 'exit_code': code,
            })
        return {'ok': True, 'message': success_message}

    @app.get('/health')
    async def health():
        return await asyncio.to_thread(inspect_robot_health, action_lock.locked())

    @app.post('/move', dependencies=[Depends(authorize)])
    async def move(request: FastAPIRequest, body: dict | None = None):
        task = {**(body if body is not None else {'fixed': True}), 'tool': 'move'}
        return await execute([task], '已到达目标位置', request)

    @app.post('/grasp', dependencies=[Depends(authorize)])
    async def grasp(request: FastAPIRequest, body: dict | None = None):
        task = {**(body or {}), 'tool': 'grasp'}
        return await execute([task], '抓放流程已完成', request)

    @app.post('/tasks', dependencies=[Depends(authorize)])
    async def run_tasks(request: FastAPIRequest, tasks: list = Body(...)):
        return await execute(tasks, '全部任务执行完成', request)

    @app.post('/cancel', dependencies=[Depends(authorize)])
    async def cancel():
        if not action_lock.locked():
            return JSONResponse(status_code=409, content={
                'ok': False, 'message': '当前没有正在执行的机器人任务',
                'cancelled': False,
            })
        scheduler.cancel_current()
        return JSONResponse(status_code=202, content={
            'ok': True, 'message': '已请求安全取消；动作锁将在控制脚本清理后释放',
            'cancelled': True,
        })

    return app


app = create_app()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1', help='监听地址；局域网访问用 0.0.0.0')
    parser.add_argument('--port', type=int, default=8000, help='监听端口，默认 8000')
    args = parser.parse_args(argv)
    if not os.environ.get('ROBOT_API_KEY'):
        parser.error('启动前必须设置 ROBOT_API_KEY')
    uvicorn.run(app, host=args.host, port=args.port, workers=1)


if __name__ == '__main__':
    main()
