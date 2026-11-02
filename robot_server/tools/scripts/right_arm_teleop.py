#!/usr/bin/python3
# 用途：交互式遥操或一次性移动 Luna 左臂/右臂的笛卡尔空间目标。
# 运行：source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh && /usr/bin/python3 tools/scripts/right_arm_teleop.py --help
# 推荐运行：source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh && /usr/bin/python3 tools/scripts/right_arm_teleop.py --side right --execute --control-backend auto
# 双臂笛卡尔阻抗：source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh && /usr/bin/python3 tools/scripts/right_arm_teleop.py --side left --execute --imp-cart --incremental
# 右臂增量式阻抗遥操：将上面命令的 --side left 改为 --side right
# 左手运行：source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh && /usr/bin/python3 tools/scripts/right_arm_teleop.py --side left --execute --control-backend auto
"""Luna 左右手均可用的笛卡尔空间 CLI 遥操。"""

import argparse
import json
import math
import os
import select
import sys
import termios
import threading
import time
import tty
import xml.etree.ElementTree as ET
from urllib.error import HTTPError, URLError
from urllib.request import ProxyHandler, Request, build_opener

import rclpy
from ament_index_python.packages import get_package_share_directory
from marvin_msgs.msg import IKRequest, Jointfeedback
from marvin_msgs.srv import Int
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32, Int16MultiArray, Int32
from tf2_ros import Buffer, TransformException, TransformListener


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--side', choices=('left', 'right'), default='right',
        help='选择控制左手或右手，默认 right；一个进程一次控制一只手')
    parser.add_argument(
        '--execute', action='store_true',
        help='允许向机器人发布控制命令；默认仅预览')
    parser.add_argument(
        '--imp-cart', action='store_true',
        help='所选手臂笛卡尔阻抗：按 E 请求双臂 mode 4，并仅在双臂状态 3 下发送目标')
    parser.add_argument(
        '--incremental', action='store_true',
        help='增量输入：位置按所选轴的实测值与已有目标推进，其他轴保持不变')
    parser.add_argument('--move-x-mm', type=float,
                        help='一次性沿 base_link X 轴移动的毫米数；指定后不进入键盘 CLI')
    parser.add_argument('--move-y-mm', type=float,
                        help='一次性沿 base_link Y 轴移动的毫米数；指定后不进入键盘 CLI')
    parser.add_argument('--move-z-mm', type=float,
                        help='一次性沿 base_link Z 轴移动的毫米数；指定后不进入键盘 CLI')
    parser.add_argument(
        '--namespace', default=os.environ.get('APEX_ROS_NAMESPACE', 'tj'),
        help='ROS 命名空间，默认 APEX_ROS_NAMESPACE 或 tj')
    parser.add_argument(
        '--control-backend', choices=('auto', 'replay', 'vlahost'), default='auto',
        help='控制链路：auto 优先 replay，无 replay 订阅者时回退 VLAHost HTTP')
    parser.add_argument(
        '--vlahost-url', default=os.environ.get('VLAHOST_URL', 'http://6.6.7.100:8011'),
        help='VLAHost HTTP 地址，默认 VLAHOST_URL 或 http://6.6.7.100:8011')
    parser.add_argument('--position-step-mm', type=float, default=2.0,
                        help='每个重复按键的位移步长，默认 2 mm')
    parser.add_argument('--angle-step-deg', type=float, default=1.0,
                        help='每个重复按键的旋转步长，默认 1 deg')
    parser.add_argument('--repeat-hz', type=float, default=12.0,
                        help='连续按键处理频率上限，默认 12 Hz')
    parser.add_argument('--max-position-offset-mm', type=float, default=0.0,
                        help='相对重置位姿的位置上限；默认 0，表示不限制')
    parser.add_argument('--max-angle-offset-deg', type=float, default=0.0,
                        help='相对重置姿态的旋转上限；默认 0，表示不限制')
    parser.add_argument('--feedback-timeout', type=float, default=0.5)
    parser.add_argument('--once-timeout-s', type=float, default=30.0,
                        help='一次性移动等待完成的超时时间，默认 30 秒')
    parser.add_argument('--once-tolerance-mm', type=float, default=8.0,
                        help='一次性移动完成的位置误差，默认 8 mm')
    parser.add_argument('--once-settle-s', type=float, default=0.3,
                        help='一次性移动达到误差后保持的确认时间，默认 0.3 秒')
    parser.add_argument('--rate-hz', type=float, default=30.0)
    parser.add_argument('--ik-source', type=int, default=1,
                        help='本程序使用的 IK mux 源，默认 replay=1')
    parser.add_argument('--joint-source', type=int, default=0,
                        help='QP 对应的 joint command mux 源，默认 0')
    parser.add_argument('--head-joint-count', type=int, default=2)
    parser.add_argument('--gripper-close-value', type=float, default=1.0,
                        help='所选夹爪的关闭命令值，默认 1.0')
    parser.add_argument('--gripper-open-value', type=float, default=0.0,
                        help='所选夹爪的打开命令值，默认 0.0')
    parser.add_argument(
        '--gripper-topic',
        help='覆盖夹爪命令话题；未指定时按命名空间话题并自动兼容 /control 话题')
    parser.add_argument('--model-xml', help='Luna MJCF 路径')
    args = parser.parse_args()
    positive = (
        'position_step_mm', 'angle_step_deg', 'feedback_timeout',
        'rate_hz', 'repeat_hz', 'once_timeout_s', 'once_tolerance_mm',
        'once_settle_s')
    for name in positive:
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f'--{name.replace("_", "-")} 必须为有限正数')
    if args.rate_hz > 100.0:
        parser.error('--rate-hz 不能大于 100')
    if (not math.isfinite(args.max_position_offset_mm) or
            args.max_position_offset_mm < 0.0):
        parser.error('--max-position-offset-mm 必须为非负有限数，0 表示不限制')
    if (not math.isfinite(args.max_angle_offset_deg) or
            args.max_angle_offset_deg < 0.0):
        parser.error('--max-angle-offset-deg 必须为非负有限数，0 表示不限制')
    if args.head_joint_count < 0 or args.head_joint_count > 3:
        parser.error('--head-joint-count 必须在 0..3 范围内')
    for name in ('gripper_close_value', 'gripper_open_value'):
        value = getattr(args, name)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            parser.error(f'--{name.replace("_", "-")} 必须在 0.0..1.0 范围内')
    move_values = (args.move_x_mm, args.move_y_mm, args.move_z_mm)
    if any(value is not None for value in move_values):
        if any(value is not None and not math.isfinite(value)
               for value in move_values):
            parser.error('--move-x-mm、--move-y-mm、--move-z-mm 必须是有限数值')
        if all(value is None or abs(value) <= 1e-12 for value in move_values):
            parser.error('一次性移动至少需要一个非零的 move-* 按毫米参数')
    return args


def topic_name(namespace, name):
    if name.startswith('/'):
        return name
    prefix = namespace.strip('/')
    return '/' + '/'.join(part for part in (prefix, name) if part)


def quaternion_normalize(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-12 or not math.isfinite(norm):
        raise ValueError('四元数无效')
    return tuple(value / norm for value in quaternion)


def quaternion_multiply(first, second):
    """按 ROS xyzw 顺序返回 first * second。"""
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2)


def rotate_vector(vector, quaternion):
    x, y, z, w = quaternion_normalize(quaternion)
    vx, vy, vz = vector
    tx = 2.0 * (y*vz - z*vy)
    ty = 2.0 * (z*vx - x*vz)
    tz = 2.0 * (x*vy - y*vx)
    return (
        vx + w*tx + y*tz - z*ty,
        vy + w*ty + z*tx - x*tz,
        vz + w*tz + x*ty - y*tx)


def axis_angle_quaternion(axis, angle_rad):
    half = angle_rad * 0.5
    values = [0.0, 0.0, 0.0]
    values[axis] = math.sin(half)
    return values[0], values[1], values[2], math.cos(half)


def quaternion_angle_deg(first, second):
    first = quaternion_normalize(first)
    second = quaternion_normalize(second)
    dot = min(1.0, abs(sum(a*b for a, b in zip(first, second))))
    return math.degrees(2.0 * math.acos(dot))


SIDE_CONFIG = {
    'left': {
        'label': '左手',
        'arm_link': 'Arm_L7_Link',
        'flange': 'flange_L',
        'tool': 'left_tool',
        'pose_field': 'left_hand_pose',
        'hand_index': 0,
        'gripper_suffix': 'L',
        'arm_states': (1, 2, 3),
    },
    'right': {
        'label': '右手',
        'arm_link': 'Arm_R7_Link',
        'flange': 'flange_R',
        'tool': 'right_tool',
        'pose_field': 'right_hand_pose',
        'hand_index': 1,
        'gripper_suffix': 'R',
        'arm_states': (1, 2, 3),
    },
}


def read_tool_mount(model_xml, side):
    """读取指定手腕连杆到对应工具坐标系的固定变换。"""
    config = SIDE_CONFIG[side]
    root = ET.parse(model_xml).getroot()
    arm = root.find(f".//body[@name='{config['arm_link']}']")
    flange = (None if arm is None else
              arm.find(f"body[@name='{config['flange']}']"))
    tool = (None if flange is None else
            flange.find(f"body[@name='{config['tool']}']"))
    if tool is None:
        raise ValueError(
            f"MJCF 中找不到 {config['arm_link']}/{config['flange']}"
            f"/{config['tool']}")

    def placement(body):
        position = tuple(float(v) for v in body.get('pos', '0 0 0').split())
        wxyz = tuple(float(v) for v in body.get('quat', '1 0 0 0').split())
        if len(position) != 3 or len(wxyz) != 4:
            raise ValueError('MJCF 工具固定变换格式错误')
        return position, (wxyz[1], wxyz[2], wxyz[3], wxyz[0])

    flange_position, flange_quaternion = placement(flange)
    tool_position, tool_quaternion = placement(tool)
    rotated = rotate_vector(tool_position, flange_quaternion)
    position = tuple(a + b for a, b in zip(flange_position, rotated))
    quaternion = quaternion_multiply(flange_quaternion, tool_quaternion)
    return position, quaternion_normalize(quaternion)


class ArmTeleop(Node):
    def __init__(self, args):
        self.side = args.side
        self.side_config = SIDE_CONFIG[self.side]
        super().__init__(f'{self.side}_arm_cartesian_teleop')
        self.args = args
        model_xml = args.model_xml
        if model_xml is None:
            model_xml = os.path.join(
                get_package_share_directory('marvin_description'),
                'mjcf', 'Gento_Luna', 'Gento_Luna.xml')
        self.tool_offset, self.tool_rotation = read_tool_mount(model_xml, self.side)
        self.other_side = 'right' if self.side == 'left' else 'left'
        self.other_tool_offset, self.other_tool_rotation = read_tool_mount(
            model_xml, self.other_side)

        self.lock = threading.RLock()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.target_position = None
        self.target_quaternion = None
        self.initial_position = None
        self.initial_quaternion = None
        self.actual_position = None
        self.actual_quaternion = None
        self.other_actual_position = None
        self.other_actual_quaternion = None
        self.actual_at = None
        self.joint_positions = []
        self.joint_feedback_at = None
        self.robot_ready = False
        self.robot_states = ()
        self.robot_state_at = None
        self.ik_source = None
        self.joint_source = None
        self.previous_ik_source = None
        self.active = False
        self.switch_pending = False
        self.control_backend = None
        self.vlahost_target = None
        self.vlahost_lock = threading.Lock()
        self.vlahost_stop = threading.Event()
        self.vlahost_wakeup = threading.Event()
        self.vlahost_thread = threading.Thread(
            target=self.vlahost_loop, name='vlahost-arm-command', daemon=True)
        self.vlahost_thread.start()
        # 不假定启动前夹爪的真实状态；按用户约定第一次空格发送打开。
        self.next_gripper_action = 'open'
        self.last_problem = '等待机器人反馈和 TF'

        request_qos = QoSProfile(depth=10)
        request_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.request_topic = topic_name(
            args.namespace, 'control/replay/ik_request')
        self.publisher = self.create_publisher(
            IKRequest, self.request_topic, request_qos)
        default_gripper_topic = topic_name(
            args.namespace,
            f"control/gripperValue{self.side_config['gripper_suffix']}")
        self.gripper_topic = (args.gripper_topic
                              if args.gripper_topic is not None
                              else default_gripper_topic)
        self.gripper_publisher = self.create_publisher(
            Float32, self.gripper_topic, 10)
        self.fallback_gripper_topic = None
        self.fallback_gripper_publisher = None
        if args.gripper_topic is None:
            self.fallback_gripper_topic = topic_name(
                '', f"control/gripperValue{self.side_config['gripper_suffix']}")
        if (self.fallback_gripper_topic is not None and
                self.fallback_gripper_topic != self.gripper_topic):
            self.fallback_gripper_publisher = self.create_publisher(
                Float32, self.fallback_gripper_topic, 10)
        self.create_subscription(
            Jointfeedback, topic_name(args.namespace, 'info/joint_feedback'),
            self.on_joint_feedback, request_qos)
        self.create_subscription(
            Int16MultiArray, topic_name(args.namespace, 'info/robot_state'),
            self.on_robot_state, 10)
        self.create_subscription(
            Int32, topic_name(
                args.namespace, 'info/ik_request_mux/active_source'),
            self.on_ik_source, latched_qos)
        self.create_subscription(
            Int32, topic_name(
                args.namespace, 'info/joint_cmd_mux/active_source'),
            self.on_joint_source, latched_qos)
        self.switch_client = self.create_client(
            Int, topic_name(args.namespace, 'control/ik_request_mux/select'))
        self.mode_client = (self.create_client(
            Int, topic_name(args.namespace, 'control/set_mode'))
            if getattr(args, 'imp_cart', False) else None)
        self.create_timer(1.0 / args.rate_hz, self.tick)

    def select_control_backend(self):
        """检测 replay 输入是否有订阅者；没有时选择 VLAHost HTTP。"""
        replay_subscribers = self.publisher.get_subscription_count()
        if self.args.control_backend == 'replay':
            if replay_subscribers == 0:
                raise ValueError(
                    f'{self.request_topic} 没有订阅者；请启动 replay 或使用 --control-backend vlahost')
            backend = 'replay'
        elif self.args.control_backend == 'vlahost':
            backend = 'vlahost'
        elif replay_subscribers > 0:
            backend = 'replay'
        else:
            backend = 'vlahost'
        if backend == 'vlahost':
            self.check_vlahost_health()
        self.control_backend = backend
        self.get_logger().info(
            f'控制链路：{backend}（{self.request_topic} 订阅者={replay_subscribers}，'
            f'VLAHost={self.args.vlahost_url}）')
        return backend

    def check_vlahost_health(self):
        """确认 VLAHost HTTP 服务可用；不发送运动命令。"""
        try:
            result = self.vlahost_request('GET', '/health')
        except Exception as error:
            raise ValueError(f'VLAHost HTTP 不可用：{error}') from error
        if not isinstance(result, dict) or result.get('status') not in (None, 'ok'):
            raise ValueError(f'VLAHost 健康检查失败：{result}')

    def vlahost_request(self, method, path, payload=None):
        url = self.args.vlahost_url.rstrip('/') + path
        data = None if payload is None else json.dumps(payload).encode('utf-8')
        request = Request(
            url, data=data, method=method,
            headers={'Content-Type': 'application/json'},
        )
        try:
            with build_opener(ProxyHandler({})).open(request, timeout=1.0) as response:
                body = response.read().decode('utf-8')
        except (HTTPError, URLError, OSError, TimeoutError) as error:
            raise RuntimeError(f'{method} {url} 失败：{error}') from error
        try:
            return json.loads(body) if body else {}
        except json.JSONDecodeError as error:
            raise RuntimeError(f'{method} {url} 返回非 JSON：{body[:200]}') from error

    def vlahost_loop(self):
        """以遥操频率发送最新目标，避免在 ROS timer 中阻塞 HTTP。"""
        period = 1.0 / max(self.args.rate_hz, 1.0)
        while not self.vlahost_stop.is_set():
            self.vlahost_wakeup.wait(timeout=period)
            self.vlahost_wakeup.clear()
            if self.vlahost_stop.is_set():
                break
            with self.lock:
                active = self.active and self.control_backend == 'vlahost'
            if not active:
                continue
            with self.vlahost_lock:
                target = self.vlahost_target
            if target is None:
                continue
            try:
                result = self.vlahost_request('POST', '/action', target)
                if not isinstance(result, dict) or not result.get('success', False):
                    with self.lock:
                        self.last_problem = f'VLAHost 动作被拒绝：{result}'
            except Exception as error:
                with self.lock:
                    self.last_problem = f'VLAHost 动作发送失败：{error}'

    def set_vlahost_target(self):
        with self.lock:
            selected_position = self.target_position
            selected_quaternion = self.target_quaternion
            other_position = self.other_actual_position
            other_quaternion = self.other_actual_quaternion
        if any(value is None for value in (
                selected_position, selected_quaternion,
                other_position, other_quaternion)):
            return
        selected_pose = [*selected_position, *selected_quaternion]
        other_pose = [*other_position, *other_quaternion]
        payload = (
            {'eef_left': selected_pose, 'eef_right': other_pose}
            if self.side == 'left' else
            {'eef_left': other_pose, 'eef_right': selected_pose}
        )
        with self.vlahost_lock:
            self.vlahost_target = payload
        self.vlahost_wakeup.set()

    def hold_vlahost_current(self):
        """释放控制前把 VLAHost 目标固定到当前双臂位姿，避免旧目标继续运行。"""
        with self.lock:
            left_position = (self.actual_position if self.side == 'left'
                             else self.other_actual_position)
            left_quaternion = (self.actual_quaternion if self.side == 'left'
                               else self.other_actual_quaternion)
            right_position = (self.other_actual_position if self.side == 'left'
                              else self.actual_position)
            right_quaternion = (self.other_actual_quaternion if self.side == 'left'
                                else self.actual_quaternion)
        if any(value is None for value in (
                left_position, left_quaternion, right_position, right_quaternion)):
            return
        payload = {
            'eef_left': [*left_position, *left_quaternion],
            'eef_right': [*right_position, *right_quaternion],
        }
        try:
            result = self.vlahost_request('POST', '/action', payload)
            if not isinstance(result, dict) or not result.get('success', False):
                self.get_logger().warning(f'VLAHost 保持当前位姿被拒绝：{result}')
        except Exception as error:
            self.get_logger().warning(f'VLAHost 保持当前位姿失败：{error}')

    def on_joint_feedback(self, message):
        count = self.args.head_joint_count
        positions = list(message.body_positions[:6])
        positions.extend(message.head_positions[:count])
        positions.extend(message.arm_positions[:14])
        with self.lock:
            self.joint_positions = positions
            self.joint_feedback_at = time.monotonic()

    def on_robot_state(self, message):
        # mode 4 会让双臂进入状态 3；普通模式下另一只手臂保持状态 1/2。
        states = tuple(message.data)
        selected_index = self.side_config['hand_index']
        other_index = 1 - selected_index
        other_states = (1, 2, 3) if self.args.imp_cart else (1, 2)
        ready = (len(states) >= 4 and
                 states[selected_index] in self.side_config['arm_states'] and
                 states[other_index] in other_states and
                 states[3] in (1, 2))
        with self.lock:
            self.robot_ready = ready
            self.robot_states = states
            self.robot_state_at = time.monotonic()

    def on_ik_source(self, message):
        with self.lock:
            self.ik_source = message.data

    def on_joint_source(self, message):
        with self.lock:
            self.joint_source = message.data

    def update_actual(self):
        stamped = self.buffer.lookup_transform(
            'base_link', self.side_config['arm_link'], Time(),
            timeout=Duration(seconds=0.01))
        stamp = stamped.header.stamp.sec + stamped.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if stamp <= 0.0 or not -0.05 <= age <= self.args.feedback_timeout:
            raise ValueError(
                f"{self.side_config['arm_link']} TF 过旧：{age:.3f} s")
        p = stamped.transform.translation
        q = stamped.transform.rotation
        arm_position = (p.x, p.y, p.z)
        arm_quaternion = quaternion_normalize((q.x, q.y, q.z, q.w))
        offset = rotate_vector(self.tool_offset, arm_quaternion)
        position = tuple(a + b for a, b in zip(arm_position, offset))
        quaternion = quaternion_normalize(
            quaternion_multiply(arm_quaternion, self.tool_rotation))
        other_link = SIDE_CONFIG[self.other_side]['arm_link']
        other_stamped = self.buffer.lookup_transform(
            'base_link', other_link, Time(), timeout=Duration(seconds=0.01))
        other_stamp = (other_stamped.header.stamp.sec +
                       other_stamped.header.stamp.nanosec * 1e-9)
        other_age = self.get_clock().now().nanoseconds * 1e-9 - other_stamp
        if (other_stamp <= 0.0 or
                not -0.05 <= other_age <= self.args.feedback_timeout):
            raise ValueError(f'{other_link} TF 过旧：{other_age:.3f} s')
        other_p = other_stamped.transform.translation
        other_q = other_stamped.transform.rotation
        other_arm_position = (other_p.x, other_p.y, other_p.z)
        other_arm_quaternion = quaternion_normalize(
            (other_q.x, other_q.y, other_q.z, other_q.w))
        other_offset = rotate_vector(
            self.other_tool_offset, other_arm_quaternion)
        other_position = tuple(
            a + b for a, b in zip(other_arm_position, other_offset))
        other_quaternion = quaternion_normalize(
            quaternion_multiply(other_arm_quaternion, self.other_tool_rotation))
        with self.lock:
            self.actual_position = position
            self.actual_quaternion = quaternion
            self.other_actual_position = other_position
            self.other_actual_quaternion = other_quaternion
            self.actual_at = time.monotonic()
            self.last_problem = None
            if self.target_position is None:
                self.target_position = position
                self.target_quaternion = quaternion
                self.initial_position = position
                self.initial_quaternion = quaternion

    def readiness_problem(self, require_imp_cart=True):
        label = self.side_config['label']
        now = time.monotonic()
        with self.lock:
            if self.target_position is None:
                return f'尚未获得{label}工具位姿'
            if (self.actual_at is None or
                    now - self.actual_at > self.args.feedback_timeout):
                return f'{label}工具 TF 反馈超时'
            if (self.robot_state_at is None or
                    now - self.robot_state_at > self.args.feedback_timeout):
                return '机器人状态反馈超时'
            if not self.robot_ready:
                return f'机器人当前不可运动，状态={self.robot_states}'
            if (require_imp_cart and self.args.imp_cart and
                    self.robot_states[:2] != (3, 3)):
                return f'双臂尚未进入笛卡尔阻抗状态 3，状态={self.robot_states}'
            if (self.joint_feedback_at is None or
                    now - self.joint_feedback_at > self.args.feedback_timeout):
                return '关节反馈超时'
            backend = self.control_backend
            if backend == 'vlahost':
                if self.other_actual_position is None:
                    return f'尚未获得{self.other_side}手工具 TF，VLAHost 需要左右位姿'
            else:
                if self.joint_source != self.args.joint_source:
                    return (f'joint mux 当前源为 {self.joint_source}，'
                            f'需要 {self.args.joint_source}')
                if self.ik_source is None:
                    return '尚未收到 IK mux 当前源'
        if self.control_backend in (None, 'replay'):
            if self.publisher.get_subscription_count() == 0:
                return f'{self.request_topic} 没有订阅者（按 E 时将自动检查 VLAHost）'
            if not self.switch_client.service_is_ready():
                return 'IK mux 切换服务不可用'
        return None

    def make_request(self, available):
        with self.lock:
            position = self.target_position
            quaternion = self.target_quaternion
            joint_positions = list(self.joint_positions)
        message = IKRequest()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.available = [False] * 11
        if available and position is not None:
            message.available[self.side_config['hand_index']] = True
            pose = getattr(message, self.side_config['pose_field'])
            pose.position.x, pose.position.y, pose.position.z = position
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w) = quaternion
        message.joint_state.position = joint_positions
        return message

    def publish_release(self):
        for _ in range(3):
            self.publisher.publish(self.make_request(False))

    def switch_source(self, source, callback=None):
        request = Int.Request()
        request.data = int(source)
        future = self.switch_client.call_async(request)

        def done(completed):
            try:
                response = completed.result()
                if response is None or not response.success:
                    text = 'IK mux 切换失败'
                    if response is not None:
                        text += f'：{response.message}'
                    with self.lock:
                        self.last_problem = text
                        self.switch_pending = False
                    return
                if callback is not None:
                    callback()
            except Exception as error:  # ROS future 会把服务错误带到这里。
                with self.lock:
                    self.last_problem = f'IK mux 服务异常：{error}'
                    self.switch_pending = False

        future.add_done_callback(done)
        return future

    def enable(self):
        if not self.args.execute:
            raise ValueError('当前是预览模式，请加 --execute 后重启')
        if self.control_backend is None:
            self.select_control_backend()
        with self.lock:
            if self.active or self.switch_pending:
                return
        if (getattr(self.args, 'imp_cart', False) and
                self.control_backend == 'replay'):
            # 切换模式前仍检查反馈和 mux；此时允许双臂尚未进入状态 3。
            problem = self.readiness_problem(require_imp_cart=False)
            if problem:
                raise ValueError(problem)
            with self.lock:
                arm_states = self.robot_states[:2]
            if arm_states != (3, 3):
                if not self.mode_client.service_is_ready():
                    raise ValueError('mode 4 切换服务不可用')
                request = Int.Request()
                request.data = 4
                future = self.mode_client.call_async(request)
                deadline = time.monotonic() + 10.0
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.02)
                if not future.done():
                    raise ValueError('mode 4 切换超时；请检查机器人状态')
                try:
                    response = future.result()
                except Exception as error:
                    raise ValueError(f'mode 4 切换异常：{error}') from error
                if response is None or not response.success:
                    detail = '' if response is None else response.message
                    raise ValueError(f'mode 4 切换失败：{detail}')
                state_deadline = time.monotonic() + 2.0
                while time.monotonic() < state_deadline:
                    with self.lock:
                        state = self.robot_states[:2]
                        fresh = (self.robot_state_at is not None and
                                 time.monotonic() - self.robot_state_at <=
                                 self.args.feedback_timeout)
                    if state == (3, 3) and fresh:
                        break
                    time.sleep(0.02)
                else:
                    raise ValueError('mode 4 服务已返回，但双臂状态 3 未确认')
        problem = self.readiness_problem()
        if problem:
            raise ValueError(problem)
        if (getattr(self.args, 'imp_cart', False) and
                self.control_backend == 'replay'):
            # 模式切换后以新鲜实测位姿接管，避免重新启用时追踪旧目标。
            self.reset_target()
        with self.lock:
            if self.active or self.switch_pending:
                return
            if self.control_backend == 'vlahost':
                self.active = True
                self.switch_pending = False
                self.last_problem = None
                self.set_vlahost_target()
                self.get_logger().warning(
                    f"{self.side_config['label']}已通过 VLAHost HTTP 启用，请保持急停可用")
                return
            source_already_selected = self.ik_source == self.args.ik_source
            # 如果上一次进程异常退出，mux 可能已经停在本程序的源；
            # 此时仍允许重新接管，释放时恢复到 teleop/idle 源 0。
            self.previous_ik_source = (
                0 if source_already_selected else self.ik_source)
            self.switch_pending = True
        publisher_count = self.count_publishers(self.request_topic)
        if publisher_count > 1:
            # gento_replay 空闲时也会注册 publisher，不能只根据 ROS 图
            # 判断它正在回放；IK mux 会选择本程序使用的 replay 源。
            self.get_logger().warning(
                f'{self.request_topic} 当前有 {publisher_count} 个发布者；'
                '请确认没有正在执行的 replay 任务')
        if rclpy.ok():
            self.publish_release()

        def activated():
            with self.lock:
                self.active = True
                self.switch_pending = False
                self.last_problem = None
            self.get_logger().warning(
                f"{self.side_config['label']}遥操已启用，请保持急停可用")

        if source_already_selected:
            activated()
        else:
            self.switch_source(self.args.ik_source, activated)

    def disable(self, restore_source=True):
        with self.lock:
            was_active = self.active or self.switch_pending
            self.active = False
            self.switch_pending = False
            previous = self.previous_ik_source
            self.previous_ik_source = None
        if self.control_backend == 'vlahost':
            with self.vlahost_lock:
                self.vlahost_target = None
            self.hold_vlahost_current()
        if rclpy.ok():
            self.publish_release()
        if (restore_source and was_active and previous is not None and
                previous != self.args.ik_source and
                self.switch_client.service_is_ready()):
            return self.switch_source(previous)
        return None

    def move(self, axis, direction):
        delta = direction * self.args.position_step_mm / 1000.0
        with self.lock:
            if self.target_position is None:
                raise ValueError('尚未获得初始位姿')
            if self.args.incremental and self.actual_position is None:
                raise ValueError('当前实际位姿不可用')
            # 只更新所选轴；目标已领先实测值时继续推进，避免连续按键停在同一目标。
            base_axis = self.target_position[axis]
            if self.args.incremental:
                actual_axis = self.actual_position[axis]
                base_axis = (max(base_axis, actual_axis) if direction > 0
                             else min(base_axis, actual_axis))
            candidate = list(self.target_position)
            candidate[axis] = base_axis + delta
            distance = math.dist(candidate, self.initial_position) * 1000.0
            if (self.args.max_position_offset_mm > 0.0 and
                    distance > self.args.max_position_offset_mm + 1e-9):
                raise ValueError(
                    f'位置偏移将超过 {self.args.max_position_offset_mm:.1f} mm')
            self.target_position = tuple(candidate)

    def rotate(self, axis, direction):
        angle = math.radians(direction * self.args.angle_step_deg)
        delta = axis_angle_quaternion(axis, angle)
        with self.lock:
            base = (self.actual_quaternion if self.args.incremental
                    else self.target_quaternion)
            if base is None:
                raise ValueError('尚未获得初始位姿')
            # 左乘表示绕固定 base_link X/Y/Z 轴旋转。
            candidate = quaternion_normalize(
                quaternion_multiply(delta, base))
            offset = quaternion_angle_deg(candidate, self.initial_quaternion)
            if (self.args.max_angle_offset_deg > 0.0 and
                    offset > self.args.max_angle_offset_deg + 1e-9):
                raise ValueError(
                    f'姿态偏移将超过 {self.args.max_angle_offset_deg:.1f} deg')
            self.target_quaternion = candidate

    def reset_target(self):
        with self.lock:
            if self.actual_position is None:
                raise ValueError('当前实际位姿不可用')
            self.target_position = self.actual_position
            self.target_quaternion = self.actual_quaternion
            self.initial_position = self.actual_position
            self.initial_quaternion = self.actual_quaternion

    def translate_once(self, delta_mm):
        """以当前实测工具位姿为基准设置一次性平移目标。"""
        delta_m = tuple(value / 1000.0 for value in delta_mm)
        with self.lock:
            if self.actual_position is None or self.initial_position is None:
                raise ValueError('当前实际位姿不可用')
            candidate = tuple(
                position + delta
                for position, delta in zip(self.actual_position, delta_m))
            distance = math.dist(candidate, self.initial_position) * 1000.0
            if (self.args.max_position_offset_mm > 0.0 and
                    distance > self.args.max_position_offset_mm + 1e-9):
                raise ValueError(
                    f'位置偏移将超过 {self.args.max_position_offset_mm:.1f} mm')
            self.target_position = candidate

    def once_delta_mm(self):
        return tuple(value or 0.0 for value in (
            self.args.move_x_mm, self.args.move_y_mm, self.args.move_z_mm))

    def command_gripper(self, value, action):
        if not self.args.execute:
            raise ValueError('当前是预览模式，请加 --execute 后重启')
        message = Float32()
        message.data = float(value)
        publisher = self.gripper_publisher
        topic = self.gripper_topic
        if publisher.get_subscription_count() == 0:
            fallback = self.fallback_gripper_publisher
            if (fallback is not None and
                    fallback.get_subscription_count() > 0):
                publisher = fallback
                topic = self.fallback_gripper_topic
            else:
                topics = self.gripper_topic
                if (self.fallback_gripper_topic is not None and
                        self.fallback_gripper_topic != self.gripper_topic):
                    topics += f' 或 {self.fallback_gripper_topic}'
                raise ValueError(
                    f'{topics} 都没有订阅者；请检查夹爪节点或使用 --gripper-topic')
        publisher.publish(message)
        self.get_logger().info(
            f"{self.side_config['label']}夹爪{action}命令已发送："
            f'{message.data:.3f}（话题：{topic}）')

    def toggle_gripper(self):
        if self.next_gripper_action == 'open':
            self.command_gripper(self.args.gripper_open_value, '打开')
            self.next_gripper_action = 'close'
            return '打开'
        self.command_gripper(self.args.gripper_close_value, '关闭')
        self.next_gripper_action = 'open'
        return '关闭'

    def tick(self):
        try:
            self.update_actual()
        except (TransformException, ValueError) as error:
            with self.lock:
                self.last_problem = (
                    f"{self.side_config['label']}工具 TF 不可用：{error}")
                active = self.active
            if active:
                self.get_logger().error(self.last_problem + '；自动释放控制')
                self.disable()
            return

        with self.lock:
            active = self.active
            ik_source = self.ik_source
        if not active:
            return
        if self.control_backend == 'vlahost':
            problem = self.readiness_problem()
            if problem:
                with self.lock:
                    self.last_problem = problem
                self.get_logger().error(problem + '；自动释放控制')
                self.disable()
                return
            self.set_vlahost_target()
            return
        if ik_source != self.args.ik_source:
            with self.lock:
                self.active = False
                self.previous_ik_source = None
                self.last_problem = (
                    f'IK mux 已被外部切换到源 {ik_source}；本程序停止发布')
            self.get_logger().warning(self.last_problem)
            return
        problem = self.readiness_problem()
        if problem:
            with self.lock:
                self.last_problem = problem
            self.get_logger().error(problem + '；自动释放控制')
            self.disable()
            return
        self.publisher.publish(self.make_request(True))


KEY_ACTIONS = {
    'UP': ('上移 Z+', lambda node: node.move(2, 1)),
    'DOWN': ('下移 Z-', lambda node: node.move(2, -1)),
    'LEFT': ('左移 Y+', lambda node: node.move(1, 1)),
    'RIGHT': ('右移 Y-', lambda node: node.move(1, -1)),
    'w': ('前进 X+', lambda node: node.move(0, 1)),
    'W': ('前进 X+', lambda node: node.move(0, 1)),
    's': ('后退 X-', lambda node: node.move(0, -1)),
    'S': ('后退 X-', lambda node: node.move(0, -1)),
    'x': ('姿态 X+', lambda node: node.rotate(0, 1)),
    'X': ('姿态 X-', lambda node: node.rotate(0, -1)),
    'y': ('姿态 Y+', lambda node: node.rotate(1, 1)),
    'Y': ('姿态 Y-', lambda node: node.rotate(1, -1)),
    'z': ('姿态 Z+', lambda node: node.rotate(2, 1)),
    'Z': ('姿态 Z-', lambda node: node.rotate(2, -1)),
}


def print_help(node):
    mode = '实机命令模式' if node.args.execute else '预览模式（不发送命令）'
    if getattr(node.args, 'imp_cart', False):
        mode += ' · 双臂笛卡尔阻抗 mode 4'
    label = node.side_config['label']
    print(f'''\nLuna {label} CLI 遥操 · {mode}
坐标轴均相对 base_link，按键立即生效，无需回车。

位置：
  ↑ / ↓       上移 Z+ / 下移 Z-
  ← / →       左移 Y+ / 右移 Y-
  W / S       前进 X+ / 后退 X-

姿态（绕固定轴）：
  x / X       绕 X 正向 / 反向
  y / Y       绕 Y 正向 / 反向
  z / Z       绕 Z 正向 / 反向

控制：
  E           启用{label}控制
  Space       切换所选夹爪开合（第一次打开，第二次关闭）
  Esc         释放控制
  r           以当前实际位姿重置目标和安全基准
  p           显示状态和目标位姿
  h           显示帮助
  q           释放控制并退出

链路：--control-backend auto 会优先使用 replay；若 replay 输入无订阅者，自动改用 VLAHost HTTP。

{'阻抗模式：按 E 请求 mode 4；Esc 仅释放遥操，不切换 SDK 模式。' if getattr(node.args, 'imp_cart', False) else ''}
{'增量输入：位置沿所选轴逐步推进，其他轴保持目标不变；姿态以实测姿态为基准。' if getattr(node.args, 'incremental', False) else ''}
步长：{node.args.position_step_mm:.1f} mm / {node.args.angle_step_deg:.1f} deg
连续按键限速：{node.args.repeat_hz:.1f} Hz
位置范围：{'不限' if node.args.max_position_offset_mm == 0 else f'{node.args.max_position_offset_mm:.1f} mm'}；
姿态范围：{'不限' if node.args.max_angle_offset_deg == 0 else f'{node.args.max_angle_offset_deg:.1f} deg'}''')


def status_snapshot(node):
    with node.lock:
        position = node.target_position
        quaternion = node.target_quaternion
        active = node.active
        pending = node.switch_pending
        ik_source = node.ik_source
        joint_source = node.joint_source
        robot_states = node.robot_states
    if active:
        state = '已启用'
    elif pending:
        state = '正在切换 IK 源'
    else:
        problem = node.readiness_problem()
        if (getattr(node.args, 'imp_cart', False) and problem and
                problem.startswith('双臂尚未进入笛卡尔阻抗状态 3') and
                node.readiness_problem(require_imp_cart=False) is None):
            state = '已释放；按 E 可请求 mode 4'
        else:
            state = '已释放；可启用' if problem is None else f'已释放；不可启用：{problem}'
    if position is None:
        pose = 'target=尚未获得'
    else:
        pose = (
            'target xyz=[{:.4f}, {:.4f}, {:.4f}] m, '
            'quat=[{:.4f}, {:.4f}, {:.4f}, {:.4f}]'.format(
                *position, *quaternion))
    return (f'{node.side_config["label"]}{state}；'
            f'backend={node.control_backend or "auto"}；'
            f'IK source={ik_source}，joint source={joint_source}，'
            f'机器人状态={robot_states}\n{pose}')


def read_key(file_descriptor, timeout=0.2):
    readable, _, _ = select.select([file_descriptor], [], [], timeout)
    if not readable:
        return None
    key = os.read(file_descriptor, 1).decode(errors='ignore')
    if key != '\x1b':
        return key
    readable, _, _ = select.select([file_descriptor], [], [], 0.03)
    if not readable:
        return 'ESC'
    second = os.read(file_descriptor, 1).decode(errors='ignore')
    if second != '[':
        return 'ESC'
    readable, _, _ = select.select([file_descriptor], [], [], 0.03)
    if not readable:
        return 'ESC'
    third = os.read(file_descriptor, 1).decode(errors='ignore')
    return {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}.get(
        third, 'ESC')


def run_once(node):
    """读取当前位姿，发送一次性平移目标，达到目标后自动释放并退出。"""
    if not node.args.execute:
        raise RuntimeError('一次性移动必须加 --execute')
    deadline = time.monotonic() + node.args.once_timeout_s
    while rclpy.ok() and time.monotonic() < deadline:
        with node.lock:
            initial_ready = (
                node.actual_position is not None and
                node.target_position is not None and
                node.robot_ready and
                node.actual_at is not None and
                node.robot_state_at is not None and
                node.joint_feedback_at is not None and
                time.monotonic() - node.actual_at <= node.args.feedback_timeout and
                time.monotonic() - node.robot_state_at <= node.args.feedback_timeout and
                time.monotonic() - node.joint_feedback_at <= node.args.feedback_timeout)
        if initial_ready:
            break
        time.sleep(0.05)
    else:
        raise RuntimeError('等待工具 TF、关节反馈或机器人状态超时')

    node.select_control_backend()
    while rclpy.ok() and time.monotonic() < deadline:
        problem = node.readiness_problem(require_imp_cart=False)
        if problem is None:
            break
        time.sleep(0.05)
    else:
        raise RuntimeError(f'一次性移动预检超时：{node.readiness_problem(require_imp_cart=False)}')

    node.reset_target()
    delta_mm = node.once_delta_mm()
    node.translate_once(delta_mm)
    with node.lock:
        goal = node.target_position
        start = node.initial_position
    node.get_logger().info(
        f'一次性移动目标：当前=({start[0]:.4f}, {start[1]:.4f}, {start[2]:.4f})，'
        f'偏移=({delta_mm[0]:.1f}, {delta_mm[1]:.1f}, {delta_mm[2]:.1f}) mm，'
        f'目标=({goal[0]:.4f}, {goal[1]:.4f}, {goal[2]:.4f})')
    node.enable()
    while rclpy.ok() and time.monotonic() < deadline:
        with node.lock:
            active = node.active
            problem = node.last_problem
        if active:
            break
        if problem:
            raise RuntimeError(f'一次性移动启用失败：{problem}')
        time.sleep(0.05)
    else:
        raise RuntimeError('一次性移动启用超时')

    reached_since = None
    while rclpy.ok() and time.monotonic() < deadline:
        with node.lock:
            actual = node.actual_position
            problem = node.last_problem
        if problem:
            raise RuntimeError(f'一次性移动过程中控制被释放：{problem}')
        error_mm = (float('inf') if actual is None else
                    math.dist(actual, goal) * 1000.0)
        if error_mm <= node.args.once_tolerance_mm:
            if reached_since is None:
                reached_since = time.monotonic()
            if time.monotonic() - reached_since >= node.args.once_settle_s:
                node.get_logger().info(
                    f'一次性移动完成：偏移=({delta_mm[0]:.1f}, {delta_mm[1]:.1f}, '
                    f'{delta_mm[2]:.1f}) mm，末端误差={error_mm:.1f} mm')
                node.disable()
                return
        else:
            reached_since = None
        time.sleep(0.05)
    raise RuntimeError('一次性移动到目标超时')


def run_interactive(node):
    if not sys.stdin.isatty():
        raise RuntimeError('CLI 遥操必须在交互式终端中运行')
    print_help(node)
    print('\n' + status_snapshot(node))
    file_descriptor = sys.stdin.fileno()
    saved_attributes = termios.tcgetattr(file_descriptor)
    last_state = None
    last_action_at = 0.0
    repeat_period = 1.0 / node.args.repeat_hz
    try:
        tty.setcbreak(file_descriptor)
        while rclpy.ok():
            with node.lock:
                state = (node.active, node.switch_pending, node.last_problem)
            if state != last_state:
                if last_state is not None:
                    print('\n' + status_snapshot(node))
                last_state = state
            key = read_key(file_descriptor)
            if key is None or key in ('\r', '\n'):
                continue
            try:
                if key in KEY_ACTIONS:
                    now = time.monotonic()
                    if now - last_action_at < repeat_period:
                        continue
                    last_action_at = now
                    label, action = KEY_ACTIONS[key]
                    action(node)
                    print(f'\n{label}：{status_snapshot(node).splitlines()[-1]}')
                elif key == ' ':
                    action = node.toggle_gripper()
                    print(f'\n{node.side_config["label"]}夹爪{action}命令已发送')
                elif key in ('e', 'E'):
                    node.enable()
                    print(f'\n正在启用{node.side_config["label"]}控制…')
                elif key == 'ESC':
                    node.disable()
                    print(f'\n已请求释放{node.side_config["label"]}控制')
                elif key in ('r', 'R'):
                    node.reset_target()
                    print('\n已将当前实际位姿设为新目标和安全基准')
                    print(status_snapshot(node))
                elif key in ('p', 'P'):
                    print('\n' + status_snapshot(node))
                elif key in ('h', 'H', '?'):
                    print_help(node)
                elif key in ('q', 'Q'):
                    print('\n正在释放控制并退出…')
                    return
            except (ValueError, RuntimeError) as error:
                print(f'\n不能执行：{error}')
    finally:
        termios.tcsetattr(file_descriptor, termios.TCSADRAIN, saved_attributes)


def main():
    args = parse_args()
    rclpy.init(args=[])
    node = None
    spin_thread = None
    try:
        node = ArmTeleop(args)
        spin_thread = threading.Thread(
            target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        if any(value is not None for value in (
                args.move_x_mm, args.move_y_mm, args.move_z_mm)):
            run_once(node)
        else:
            run_interactive(node)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(f'right_arm_teleop: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.disable()
            time.sleep(0.05)
            node.vlahost_stop.set()
            node.vlahost_wakeup.set()
            node.vlahost_thread.join(timeout=1.0)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    sys.exit(main())
