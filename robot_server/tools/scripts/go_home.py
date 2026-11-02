#!/usr/bin/env python3
# 用途：通过 ROS Trigger 服务调用机器人 control/go_home，恢复机器人 home；运行：source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh && /usr/bin/python3 tools/scripts/go_home.py。
"""通过 ROS 服务执行机器人 go home，不访问 HTTP 或 VLAHost。"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import time

try:  # --help 和离线预览不要求已加载 ROS 环境。
    import rclpy
    from std_srvs.srv import Trigger
except ImportError as error:  # pragma: no cover - 取决于运行环境是否加载 ROS
    rclpy = None
    Trigger = None
    ROS_IMPORT_ERROR = error
else:
    ROS_IMPORT_ERROR = None


def topic_name(namespace, name):
    """将相对服务名解析到 ROS 命名空间；绝对服务名保持不变。"""
    if name.startswith('/'):
        return name
    prefix = namespace.strip('/')
    return '/' + '/'.join(part for part in (prefix, name.strip('/')) if part)


def started_duration(message):
    """解析 QP 控制器返回的 go_home 计划时长。"""
    match = re.search(r'\bGo home started, duration=([0-9]+(?:\.[0-9]+)?)s\b', message)
    if match is None:
        raise ValueError(f'go_home 未返回计划时长，无法确认何时继续后续任务：{message}')
    duration = float(match.group(1))
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError(f'go_home 返回无效计划时长：{message}')
    return duration


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--namespace', default=os.environ.get('APEX_ROS_NAMESPACE', 'tj'),
        help='ROS 命名空间，默认 APEX_ROS_NAMESPACE 或 tj')
    parser.add_argument(
        '--service', default='control/go_home',
        help='go home 服务名；相对名称位于 namespace 下，默认 control/go_home')
    parser.add_argument(
        '--wait-timeout-s', type=float, default=30.0,
        help='等待服务和响应的超时时间（秒），默认 30')
    parser.add_argument(
        '--completion-margin-s', type=float, default=0.5,
        help='计划时长后的额外等待时间（秒），默认 0.5')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if (not math.isfinite(args.wait_timeout_s) or args.wait_timeout_s <= 0.0 or
            not math.isfinite(args.completion_margin_s) or args.completion_margin_s < 0.0):
        print('--wait-timeout-s 必须为有限正数；--completion-margin-s 必须为有限非负数',
              file=sys.stderr)
        return 2
    if rclpy is None:
        print(f'无法导入 ROS rclpy/std_srvs，请先加载 ROS 环境：{ROS_IMPORT_ERROR}',
              file=sys.stderr)
        return 2

    service = topic_name(args.namespace, args.service)
    node = None
    initialized = False
    try:
        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node('ros_go_home_tool')
        client = node.create_client(Trigger, service)
        node.get_logger().info(f'等待 ROS go home 服务 {service}')
        if not client.wait_for_service(timeout_sec=args.wait_timeout_s):
            node.get_logger().error(f'等待 ROS 服务超时：{service}')
            return 2
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=args.wait_timeout_s)
        if not future.done():
            node.get_logger().error(f'ROS go home 响应超时：{service}')
            return 2
        response = future.result()
        if response is None:
            node.get_logger().error('ROS go home 服务没有返回响应')
            return 2
        message = str(response.message).strip()
        if not response.success:
            node.get_logger().error(
                f'ROS go home 失败：{message or "服务返回 success=false"}')
            return 1
        duration = started_duration(message)
        node.get_logger().info(f'ROS go home 已启动；等待计划时长 {duration:.2f} 秒')
        time.sleep(duration + args.completion_margin_s)
        node.get_logger().info('ROS go home 计划时长已结束，可执行下一任务')
        return 0
    except (RuntimeError, ValueError) as error:
        if node is not None:
            node.get_logger().error(f'ROS go home 异常：{error}')
        else:
            print(f'ROS go home 异常：{error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
