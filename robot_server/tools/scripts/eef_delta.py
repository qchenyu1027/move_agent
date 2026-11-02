#!/usr/bin/env python3
# 用途：按 RPent Luna EEF 逻辑，通过 VLAHost action_chunk 执行一次有界平移。
"""Execute one selected EEF translation through the RPent VLAHost contract.

The script deliberately does not publish IK targets and does not use the
keyboard teleoperation implementation.  It reads the selected tool from local
TF, verifies it against ``/hil/snapshot``, sends both absolute EEF poses in
10-frame ``/action_chunk`` requests, and checks the published chunk before
accepting the motion.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET
from urllib.error import HTTPError, URLError
from urllib.request import ProxyHandler, Request, build_opener

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int16MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


DEFAULT_VLAHOST_URL = 'http://6.6.7.100:8011'
DEFAULT_NAMESPACE = os.environ.get('APEX_ROS_NAMESPACE', 'tj')
FEEDBACK_TIMEOUT_S = 0.5
TF_FUTURE_SKEW_S = 1.0
SNAPSHOT_TIMEOUT_S = 3.0
CHUNK_FRAMES = 10
CHUNK_SOURCE_HZ = 20.0
CHUNK_DURATION_S = CHUNK_FRAMES / CHUNK_SOURCE_HZ
OTHER_TOOL_MAX_DRIFT_M = 0.02
LOCAL_VLA_POSITION_TOLERANCE_M = 0.015
LOCAL_VLA_ANGLE_TOLERANCE_DEG = 5.0


SIDE_CONFIG = {
    'left': {
        'arm_link': 'Arm_L7_Link',
        'flange': 'flange_L',
        'tool': 'left_tool',
    },
    'right': {
        'arm_link': 'Arm_R7_Link',
        'flange': 'flange_R',
        'tool': 'right_tool',
    },
}


def topic_name(namespace, name):
    if name.startswith('/'):
        return name
    return '/' + '/'.join(part for part in (namespace.strip('/'), name) if part)


def normalize_quaternion(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('四元数无效')
    return tuple(value / norm for value in quaternion)


def angular_error_deg(first, second):
    first = normalize_quaternion(first)
    second = normalize_quaternion(second)
    dot = min(1.0, abs(sum(a * b for a, b in zip(first, second))))
    return math.degrees(2.0 * math.acos(dot))


def multiply_quaternions(first, second):
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def rotate_vector(vector, quaternion):
    x, y, z, w = normalize_quaternion(quaternion)
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def read_tool_mount(model_xml, side):
    config = SIDE_CONFIG[side]
    root = ET.parse(model_xml).getroot()
    arm = root.find(f".//body[@name='{config['arm_link']}']")
    flange = None if arm is None else arm.find(f"body[@name='{config['flange']}']")
    tool = None if flange is None else flange.find(f"body[@name='{config['tool']}']")
    if tool is None:
        raise ValueError(
            f"MJCF 中找不到 {config['arm_link']}/{config['flange']}/{config['tool']}"
        )

    def placement(body):
        position = tuple(float(value) for value in body.get('pos', '0 0 0').split())
        wxyz = tuple(float(value) for value in body.get('quat', '1 0 0 0').split())
        if len(position) != 3 or len(wxyz) != 4:
            raise ValueError('MJCF 工具固定变换格式错误')
        return position, normalize_quaternion((wxyz[1], wxyz[2], wxyz[3], wxyz[0]))

    flange_position, flange_quaternion = placement(flange)
    tool_position, tool_quaternion = placement(tool)
    position = tuple(
        a + b for a, b in zip(flange_position, rotate_vector(tool_position, flange_quaternion))
    )
    return position, multiply_quaternions(flange_quaternion, tool_quaternion)


def request_json(base_url, path, body=None, timeout=SNAPSHOT_TIMEOUT_S):
    url = base_url.rstrip('/') + path
    if body is None:
        request = Request(url, method='GET')
    else:
        request = Request(
            url,
            data=json.dumps(body).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
    try:
        with build_opener(ProxyHandler({})).open(request, timeout=timeout) as response:
            result = json.load(response)
    except HTTPError as error:
        detail = error.read().decode('utf-8', errors='replace')
        raise ValueError(f'VLAHost {path} HTTP {error.code}: {detail}') from error
    except (URLError, OSError, json.JSONDecodeError) as error:
        raise ValueError(f'VLAHost {path} 不可用：{error}') from error
    if not isinstance(result, dict):
        raise ValueError(f'VLAHost {path} 返回的不是 JSON 对象')
    return result


def _remote_pose(snapshot, side):
    pose = snapshot.get(f'eef_{side}')
    if not isinstance(pose, dict) or pose.get('frame_id') != 'base_link':
        raise ValueError(f'VLAHost eef_{side} 位姿缺失或不在 base_link')
    position = pose.get('position')
    orientation = pose.get('orientation')
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise ValueError(f'VLAHost eef_{side} 位姿格式无效')
    values = [position.get(axis) for axis in ('x', 'y', 'z')]
    values += [orientation.get(axis) for axis in ('x', 'y', 'z', 'w')]
    if any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        for value in values
    ):
        raise ValueError(f'VLAHost eef_{side} 位姿包含无效数值')
    quaternion = normalize_quaternion(tuple(values[3:]))
    if abs(math.sqrt(sum(value * value for value in values[3:])) - 1.0) > 0.02:
        raise ValueError(f'VLAHost eef_{side} 四元数未归一化')
    return [*map(float, values[:3]), *quaternion]


def eef_snapshot(base_url):
    snapshot = request_json(base_url, '/hil/snapshot')
    if snapshot.get('connected') is not True:
        raise ValueError('VLAHost 没有机器人反馈')
    capabilities = snapshot.get('capabilities', {})
    if capabilities.get('eef_action_format') != 'absolute_xyz_xyzw_per_arm':
        raise ValueError('VLAHost 不支持双臂绝对 EEF 控制')
    if capabilities.get('eef_frame_id') != 'base_link':
        raise ValueError('VLAHost EEF 坐标系不是 base_link')
    intervention = snapshot.get('intervention', {})
    if intervention.get('session_enabled') or intervention.get('footkey'):
        raise ValueError('VLAHost 干预会话正在占用控制权')
    freshness = snapshot.get('freshness', {})
    poses = {}
    for side in ('left', 'right'):
        side_freshness = freshness.get(f'eef_{side}', {})
        age = side_freshness.get('age_s')
        source_age = side_freshness.get('source_age_s')
        if (
            not isinstance(age, (int, float))
            or not 0.0 <= age <= FEEDBACK_TIMEOUT_S
            or not isinstance(source_age, (int, float))
            or not -0.1 <= source_age <= FEEDBACK_TIMEOUT_S
        ):
            raise ValueError(f'VLAHost eef_{side} 位姿过期或缺失')
        poses[side] = _remote_pose(snapshot, side)
    return snapshot, poses


class EefStateNode(Node):
    """Read-only local TF and robot-state gate; never publishes IK commands."""

    def __init__(self, args):
        super().__init__('rpent_eef_delta')
        self.args = args
        self.side = args.side
        self.arm_link = SIDE_CONFIG[args.side]['arm_link']
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        model_xml = args.model_xml
        if model_xml is None:
            model_xml = os.path.join(
                get_package_share_directory('marvin_description'),
                'mjcf', 'Gento_Luna', 'Gento_Luna.xml')
        self.mount_position, self.mount_quaternion = read_tool_mount(
            model_xml, args.side)
        self.robot_ready = False
        self.robot_state_at = None
        self.create_subscription(
            Int16MultiArray,
            topic_name(args.namespace, 'info/robot_state'),
            self._on_robot_state,
            10,
        )

    def _on_robot_state(self, message):
        self.robot_ready = (
            len(message.data) >= 4
            and all(message.data[index] in (1, 2) for index in (0, 1, 3))
        )
        self.robot_state_at = time.monotonic()

    def get_actual(self):
        try:
            stamped = self.buffer.lookup_transform(
                'base_link', self.arm_link, Time(), timeout=Duration(seconds=0.02)
            )
        except TransformException as error:
            raise ValueError(f'{self.arm_link} TF 不可用：{error}') from error
        stamp = stamped.header.stamp.sec + stamped.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if stamp <= 0.0 or not -TF_FUTURE_SKEW_S <= age <= FEEDBACK_TIMEOUT_S:
            raise ValueError(f'{self.arm_link} TF 时间戳无效或过旧：{age:.3f} s')
        translation = stamped.transform.translation
        rotation = stamped.transform.rotation
        arm_position = (translation.x, translation.y, translation.z)
        arm_quaternion = normalize_quaternion(
            (rotation.x, rotation.y, rotation.z, rotation.w)
        )
        tool_position = tuple(
            a + b for a, b in zip(
                arm_position, rotate_vector(self.mount_position, arm_quaternion)
            )
        )
        tool_quaternion = normalize_quaternion(
            multiply_quaternions(arm_quaternion, self.mount_quaternion)
        )
        return tool_position, tool_quaternion


def wait_for_ready(node, args):
    deadline = time.monotonic() + args.tf_timeout_s
    last_error = '等待机器人状态和 EEF TF'
    while rclpy.ok() and time.monotonic() < deadline:
        now = time.monotonic()
        if (
            node.robot_state_at is not None
            and now - node.robot_state_at <= FEEDBACK_TIMEOUT_S
            and node.robot_ready
        ):
            try:
                actual = node.get_actual()
                return actual
            except ValueError as error:
                last_error = str(error)
        else:
            last_error = '机器人状态反馈过期或当前不可运动'
        time.sleep(0.05)
    raise RuntimeError(f'等待 RPent EEF 预检数据超时：{last_error}')


def execute_vlahost_eef(node, args, delta_mm):
    if node.robot_state_at is None or time.monotonic() - node.robot_state_at > FEEDBACK_TIMEOUT_S:
        raise ValueError('机器人状态反馈过期')
    if not node.robot_ready:
        raise ValueError('机器人当前不可运动')
    snapshot, poses = eef_snapshot(args.vlahost_url)
    ik_topic = topic_name(args.namespace, 'control/ik_request/vla')
    publisher_count = node.count_publishers(ik_topic)
    if publisher_count != 1:
        raise ValueError(
            f'RPent EEF 控制要求 {ik_topic} 恰好一个发布者，当前为 {publisher_count}'
        )
    if snapshot.get('need_new_chunk') is not True:
        raise ValueError('VLAHost 当前仍有未释放的 action chunk')
    actual_xyz, actual_quaternion = node.get_actual()
    current = poses[node.side]
    if (
        math.dist(actual_xyz, current[:3]) > LOCAL_VLA_POSITION_TOLERANCE_M
        or angular_error_deg(actual_quaternion, current[3:])
        > LOCAL_VLA_ANGLE_TOLERANCE_DEG
    ):
        raise ValueError('VLAHost EEF 位姿与本地 TF 不一致')
    target = [
        actual_xyz[index] + delta_mm[index] / 1000.0 for index in range(3)
    ] + list(actual_quaternion)
    other = 'right' if node.side == 'left' else 'left'
    other_hold = poses[other]
    deadline = time.monotonic() + args.motion_timeout_s
    settled_at = None
    last_error = float('inf')
    while rclpy.ok() and time.monotonic() < deadline:
        if (
            node.robot_state_at is None
            or time.monotonic() - node.robot_state_at > FEEDBACK_TIMEOUT_S
            or not node.robot_ready
        ):
            raise ValueError('机器人运动状态在 EEF 动作期间失效')
        snapshot, poses = eef_snapshot(args.vlahost_url)
        if snapshot.get('need_new_chunk') is not True:
            raise ValueError('VLAHost action chunk 尚未释放')
        if math.dist(poses[other][:3], other_hold[:3]) > OTHER_TOOL_MAX_DRIFT_M:
            raise ValueError(f'未选中的 {other} EEF 移动超过 20 mm')
        action = {
            'eef_left': target if node.side == 'left' else other_hold,
            'eef_right': target if node.side == 'right' else other_hold,
        }
        reply = request_json(
            args.vlahost_url,
            '/action_chunk',
            {'actions': [action] * CHUNK_FRAMES, 'source_hz': CHUNK_SOURCE_HZ},
        )
        if (
            reply.get('success') is not True
            or reply.get('execution') != 'active'
            or not isinstance(reply.get('sequence'), int)
        ):
            raise ValueError(f'VLAHost 拒绝 EEF action chunk：{reply}')
        time.sleep(CHUNK_DURATION_S + 0.05)
        snapshot, _ = eef_snapshot(args.vlahost_url)
        publication = snapshot.get('publication', {})
        intervention = snapshot.get('intervention', {})
        if (
            publication.get('sequence') != reply['sequence']
            or publication.get('action_index') != CHUNK_FRAMES - 1
            or intervention.get('ik_source') != 2
            or intervention.get('input_mode') != 0
        ):
            raise ValueError('VLAHost 未完整发布 EEF chunk，或 IK/QP 路由异常')
        actual_xyz, actual_quaternion = node.get_actual()
        last_error = math.dist(actual_xyz, target[:3])
        if last_error <= args.tolerance_mm / 1000.0:
            settled_at = settled_at or time.monotonic()
            if time.monotonic() - settled_at >= args.settle_s:
                return actual_xyz, actual_quaternion, last_error
        else:
            settled_at = None
    raise RuntimeError(
        f'RPent EEF 未在 {args.motion_timeout_s:g} s 内到位，'
        f'误差={last_error * 1000:.1f} mm'
    )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right'), required=True)
    parser.add_argument('--dx-mm', type=float, required=True)
    parser.add_argument('--dy-mm', type=float, required=True)
    parser.add_argument('--dz-mm', type=float, required=True)
    parser.add_argument('--execute', action='store_true', help='允许真实运动')
    parser.add_argument('--namespace', default=DEFAULT_NAMESPACE)
    parser.add_argument('--vlahost-url', default=DEFAULT_VLAHOST_URL)
    parser.add_argument('--model-xml')
    parser.add_argument('--tf-timeout-s', type=float, default=5.0)
    parser.add_argument('--motion-timeout-s', type=float, default=20.0)
    parser.add_argument('--tolerance-mm', type=float, default=7.0)
    parser.add_argument('--settle-s', type=float, default=0.5)
    parser.add_argument('--max-position-offset-mm', type=float, default=200.0)
    args = parser.parse_args(argv)
    values = (args.dx_mm, args.dy_mm, args.dz_mm)
    if any(not math.isfinite(value) for value in values):
        parser.error('dx/dy/dz 必须是有限数值')
    if not any(values):
        parser.error('EEF 位移不能为零')
    if math.dist(values, (0.0, 0.0, 0.0)) > 200.0 + 1e-9:
        parser.error('EEF 位移向量范数不能超过 200 mm')
    for name in ('tf_timeout_s', 'motion_timeout_s', 'tolerance_mm', 'settle_s'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0.0:
            parser.error(f'--{name.replace("_", "-")} 必须是有限正数')
    if (
        not math.isfinite(args.max_position_offset_mm)
        or not 0.0 < args.max_position_offset_mm <= 200.0
    ):
        parser.error('--max-position-offset-mm 必须在 (0, 200] mm 内')
    if math.dist(values, (0.0, 0.0, 0.0)) > args.max_position_offset_mm + 1e-9:
        parser.error('EEF 位移超过相对起始位姿上限')
    return args


def main(argv=None):
    args = parse_args(argv)
    if not args.execute:
        print('EEF 真实运动必须显式指定 --execute', file=sys.stderr)
        return 2
    rclpy.init(args=[])
    node = None
    spinner = None
    try:
        node = EefStateNode(args)
        spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spinner.start()
        wait_for_ready(node, args)
        actual_xyz, _, error_m = execute_vlahost_eef(
            node, args, (args.dx_mm, args.dy_mm, args.dz_mm)
        )
        print(json.dumps({
            'ok': True,
            'eef_side': args.side,
            'delta_mm': [args.dx_mm, args.dy_mm, args.dz_mm],
            'actual_xyz_m': list(actual_xyz),
            'error_mm': error_m * 1000.0,
            'control': 'rpent_vlahost_action_chunk',
        }, ensure_ascii=False), flush=True)
        return 0
    except (RuntimeError, ValueError, OSError) as error:
        print(f'rpent_eef_delta: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if spinner is not None:
            spinner.join(timeout=1.0)


if __name__ == '__main__':
    sys.exit(main())
