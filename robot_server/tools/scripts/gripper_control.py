#!/usr/bin/env python3
# 用途：通过 ROS Float32 话题或 VLAHost HTTP 控制单侧夹爪开合，并按指定速度平滑到目标开合度；运行：/usr/bin/python3 tools/scripts/gripper_control.py --side left --open --speed 0.5。
"""ROS/VLAHost 夹爪开合控制工具。

开合度按驱动器约定使用 0.0 到 1.0；0/1 对应全闭/全开仍需结合现场夹爪
安装方向确认。``--speed`` 是开合度每秒的变化量，例如 0.5 表示每秒变化
0.5 个归一化单位。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import ProxyHandler, Request, build_opener

try:  # --help 在未加载 ROS 环境时也应可用。
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float32, Float32MultiArray
except ImportError as error:  # pragma: no cover - 取决于运行环境
    rclpy = None
    Float32 = Float32MultiArray = None
    qos_profile_sensor_data = None
    ROS_IMPORT_ERROR = error
else:
    ROS_IMPORT_ERROR = None


def topic_name(namespace, name):
    if name.startswith('/'):
        return name
    prefix = namespace.strip('/')
    return '/' + '/'.join(part for part in (prefix, name.strip('/')) if part)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right'), required=True)
    parser.add_argument('--control-backend', choices=('ros', 'vlahost'), default='ros',
                        help='控制链路，默认 ros；可选 vlahost HTTP')
    parser.add_argument('--vlahost-url', default=os.environ.get(
        'VLAHOST_URL', 'http://6.6.7.100:8011'))
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--target-position', type=float, help='目标开合度 0..1')
    action.add_argument('--open', action='store_true', help='移动到 --open-position')
    action.add_argument('--close', action='store_true', help='移动到 --close-position')
    parser.add_argument('--open-position', type=float, default=1.0)
    parser.add_argument('--close-position', type=float, default=0.0)
    parser.add_argument('--speed', type=float, default=1.0,
                        help='开合速度，单位为开合度/秒，默认 1.0')
    parser.add_argument('--publish-hz', type=float, default=50.0,
                        help='命令发布频率，默认 50 Hz')
    parser.add_argument('--feedback-timeout-s', type=float, default=3.0,
                        help='等待反馈和夹爪订阅者的超时时间，默认 3 秒')
    parser.add_argument('--start-position', type=float,
                        help='可选起始开合度；不提供时读取反馈 position[0]')
    parser.add_argument('--hold-s', type=float, default=0.2,
                        help='到达目标后继续发布的时间，默认 0.2 秒')
    parser.add_argument('--namespace', default=os.environ.get('APEX_ROS_NAMESPACE', 'tj'))
    parser.add_argument('--topic', help='覆盖命令话题')
    parser.add_argument('--feedback-topic', help='覆盖反馈话题')
    args = parser.parse_args(argv)
    for name in ('open_position', 'close_position', 'speed', 'publish_hz',
                 'feedback_timeout_s', 'hold_s'):
        value = getattr(args, name)
        if not math.isfinite(value):
            parser.error(f'--{name.replace("_", "-")} 必须是有限数值')
    for name in ('open_position', 'close_position'):
        value = getattr(args, name)
        if not 0.0 <= value <= 1.0:
            parser.error(f'--{name.replace("_", "-")} 必须在 0..1 范围内')
    if args.target_position is not None and not 0.0 <= args.target_position <= 1.0:
        parser.error('--target-position 必须在 0..1 范围内')
    if args.start_position is not None and not 0.0 <= args.start_position <= 1.0:
        parser.error('--start-position 必须在 0..1 范围内')
    if args.speed <= 0.0 or args.publish_hz <= 0.0 or args.feedback_timeout_s <= 0.0:
        parser.error('--speed、--publish-hz、--feedback-timeout-s 必须为正数')
    if args.hold_s < 0.0:
        parser.error('--hold-s 必须为非负数')
    if args.open:
        args.target = args.open_position
    elif args.close:
        args.target = args.close_position
    else:
        args.target = args.target_position
    return args


class GripperController:
    def __init__(self, args):
        suffix = 'L' if args.side == 'left' else 'R'
        self.args = args
        self.command_topic = args.topic or topic_name(
            args.namespace, f'control/gripperValue{suffix}')
        self.feedback_topic = args.feedback_topic or topic_name(
            args.namespace, f'info/gripper_feedback_{suffix}')
        self.node = rclpy.create_node(f'{args.side}_gripper_control')
        self.publisher = self.node.create_publisher(Float32, self.command_topic, 10)
        self.feedback = None
        self.node.create_subscription(
            Float32MultiArray, self.feedback_topic, self.on_feedback,
            qos_profile_sensor_data)

    def on_feedback(self, message):
        if message.data and math.isfinite(float(message.data[0])):
            self.feedback = min(1.0, max(0.0, float(message.data[0])))

    def wait_ready(self):
        deadline = time.monotonic() + self.args.feedback_timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.publisher.get_subscription_count() > 0:
                if self.args.start_position is not None or self.feedback is not None:
                    return
        missing = []
        if self.publisher.get_subscription_count() == 0:
            missing.append(f'命令话题 {self.command_topic} 没有订阅者')
        if self.args.start_position is None and self.feedback is None:
            missing.append(f'反馈话题 {self.feedback_topic} 没有有效 position[0]')
        raise RuntimeError('；'.join(missing))

    def publish(self, value):
        self.publisher.publish(Float32(data=float(value)))

    def run(self):
        self.wait_ready()
        start = (self.args.start_position
                 if self.args.start_position is not None else self.feedback)
        target = self.args.target
        duration = abs(target - start) / self.args.speed
        period = 1.0 / self.args.publish_hz
        started = time.monotonic()
        while True:
            elapsed = time.monotonic() - started
            ratio = 1.0 if duration <= 0.0 else min(1.0, elapsed / duration)
            self.publish(start + (target - start) * ratio)
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if ratio >= 1.0:
                break
            time.sleep(period)
        hold_deadline = time.monotonic() + self.args.hold_s
        while time.monotonic() < hold_deadline:
            self.publish(target)
            rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(period)
        self.node.get_logger().info(
            f'{self.args.side} 夹爪完成：{start:.3f} -> {target:.3f}，'
            f'速度={self.args.speed:.3f}/s，话题={self.command_topic}')


def vlahost_request(args, method, path, payload=None):
    url = args.vlahost_url.rstrip('/') + path
    data = None if payload is None else json.dumps(payload).encode('utf-8')
    request = Request(url, data=data, method=method,
                      headers={'Content-Type': 'application/json'})
    try:
        with build_opener(ProxyHandler({})).open(request, timeout=1.0) as response:
            body = response.read().decode('utf-8')
    except (HTTPError, URLError, OSError, TimeoutError) as error:
        raise RuntimeError(f'{method} {url} 失败：{error}') from error
    try:
        return json.loads(body) if body else {}
    except json.JSONDecodeError as error:
        raise RuntimeError(f'{method} {url} 返回非 JSON：{body[:200]}') from error


def run_vlahost(args):
    """通过 VLAHost /action 发送夹爪目标；速度由客户端逐帧插值实现。"""
    health = vlahost_request(args, 'GET', '/health')
    if not isinstance(health, dict) or health.get('ok') is False:
        raise RuntimeError(f'VLAHost 健康检查失败：{health}')
    if args.start_position is None:
        state = vlahost_request(args, 'GET', '/state')
        field = f'gripper_{args.side}'
        feedback = state.get(field) if isinstance(state, dict) else None
        if not isinstance(feedback, dict) or not math.isfinite(float(feedback.get('position'))):
            raise RuntimeError(f'VLAHost /state 没有有效的 {field}.position')
        start = min(1.0, max(0.0, float(feedback['position'])))
    else:
        start = args.start_position
    target = args.target
    duration = abs(target - start) / args.speed
    period = 1.0 / args.publish_hz
    field = f'gripper_{args.side}'
    started = time.monotonic()
    while True:
        elapsed = time.monotonic() - started
        ratio = 1.0 if duration <= 0.0 else min(1.0, elapsed / duration)
        value = start + (target - start) * ratio
        result = vlahost_request(args, 'POST', '/action', {field: value})
        if not isinstance(result, dict) or not result.get('success', False):
            raise RuntimeError(f'VLAHost 夹爪命令被拒绝：{result}')
        if ratio >= 1.0:
            break
        time.sleep(period)
    hold_deadline = time.monotonic() + args.hold_s
    while time.monotonic() < hold_deadline:
        result = vlahost_request(args, 'POST', '/action', {field: target})
        if not isinstance(result, dict) or not result.get('success', False):
            raise RuntimeError(f'VLAHost 夹爪保持命令被拒绝：{result}')
        time.sleep(period)
    print(f'{args.side} 夹爪完成（VLAHost）：{start:.3f} -> {target:.3f}，'
          f'速度={args.speed:.3f}/s，地址={args.vlahost_url}')


def main(argv=None):
    args = parse_args(argv)
    if args.control_backend == 'vlahost':
        try:
            run_vlahost(args)
            return 0
        except (RuntimeError, ValueError) as error:
            print(f'夹爪控制失败：{error}', file=sys.stderr)
            return 1
    if rclpy is None:
        print(f'无法导入 ROS rclpy/std_msgs，请先加载 ROS 环境：{ROS_IMPORT_ERROR}',
              file=sys.stderr)
        return 2
    node = None
    initialized = False
    try:
        rclpy.init(args=[])
        initialized = True
        controller = GripperController(args)
        node = controller.node
        controller.run()
        return 0
    except (RuntimeError, ValueError) as error:
        print(f'夹爪控制失败：{error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
