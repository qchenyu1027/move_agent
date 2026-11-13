#!/usr/bin/python3
# 用途：接收头部视觉发布的物体 PoseStamped，转换到 base_link，选左臂或右臂抓取并抬升。
# 预览：source /home/liuqian/copy/Apex_Deploy/apex_ros_env.sh && PYTHONNOUSERSITE=1 /usr/bin/python3 head_object_grasp.py --side left --object-pose-topic /object_pose --pose-request-topic /f2g_pose/request
# 实机：source /home/liuqian/copy/Apex_Deploy/apex_ros_env.sh && PYTHONNOUSERSITE=1 /usr/bin/python3 head_object_grasp.py --side left --object-pose-topic /object_pose --pose-request-topic /f2g_pose/request --execute --control-backend joint
# 只到位置并缓慢闭合：source /home/liuqian/copy/Apex_Deploy/apex_ros_env.sh && /usr/bin/python3 head_object_grasp.py --side left --object-pose-topic /object_pose --pose-request-topic /f2g_pose/request --execute --position-only --gripper-duration 12
# 远程 VLAHost HTTP：在上述实机命令末尾增加 --control-backend http --vlahost-url http://6.6.7.100:8011 --http-log-hz 10
"""头部相机物体位置和姿态抓取。

物体 PoseStamped 的姿态通过固定的瓶子坐标系到左工具坐标系补偿，
目标工具坐标约定为 X 前后、Y 左右、Z 上下。

使用 ``--position-only`` 时，水瓶姿态不参与目标计算和稳定性判定，
工具保持进入执行阶段时的当前姿态，只移动到水瓶位置。

视觉程序在物体位置明显变化时会发布 ``/object_pose_changed``（std_msgs/Bool）。
本脚本在采集、预检、接近或下降阶段收到该通知后，会先以当前反馈保持位姿，
丢弃已冻结的物体位姿，再按新的稳定位置重新规划和执行；闭合夹爪和抬升阶段
忽略该通知，避免把机器人自身移动水瓶误判为物体被挪动。
"""

import argparse
from collections import deque
import json
import math
import os
import sys
import time
import numpy as np
from urllib.error import HTTPError, URLError
from urllib.request import ProxyHandler, Request, build_opener

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from marvin_msgs.msg import IKRequest, JointcmdArm, JointcmdBody, JointcmdHead, Jointfeedback
from marvin_msgs.srv import Int
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import (Bool, Empty, Float32, Float32MultiArray, Int16MultiArray, Int32,
                          Int32MultiArray)
from tf2_ros import Buffer, TransformException, TransformListener

from left_tool_accuracy_test import (angular_error_deg, multiply_quaternions,
                                     read_tool_mount, rotate_vector, topic_name,
                                     transform_pose)

try:
    import pinocchio as pin
except ImportError:
    pin = None

USER_JOINT_SOURCE = 2


def normalize(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('工具姿态四元数无效')
    return tuple(value / norm for value in quaternion)


def smoothstep01(value):
    """将 0..1 的闭合进度变成平滑的起停曲线，减少夹爪冲击。"""
    progress = min(1.0, max(0.0, float(value)))
    return progress * progress * (3.0 - 2.0 * progress)


def http_json(base_url, path, body=None, timeout=2.0):
    """调用 VLAHost JSON 接口；禁用环境代理，避免内网地址走代理。"""
    url = base_url.rstrip('/') + path
    if body is None:
        request = Request(url, method='GET')
    else:
        request = Request(
            url, data=json.dumps(body).encode('utf-8'),
            headers={'Content-Type': 'application/json'}, method='POST')
    try:
        with build_opener(ProxyHandler({})).open(request, timeout=timeout) as response:
            value = json.load(response)
    except HTTPError as error:
        detail = error.read().decode('utf-8', errors='replace')
        raise ValueError(f'VLAHost HTTP {error.code}: {detail}') from error
    except (URLError, OSError, json.JSONDecodeError) as error:
        raise ValueError(f'无法访问 VLAHost {url}: {error}') from error
    if not isinstance(value, dict):
        raise ValueError(f'VLAHost {path} 响应不是 JSON 对象')
    if value.get('success') is False:
        raise ValueError(f'VLAHost {path} 失败：{value.get("message", value)}')
    return value


def remote_pose(state, side):
    """从 VLAHost /state 的 eef_* 字段提取 xyz+xyzw。"""
    pose = state.get(f'eef_{side}')
    if not isinstance(pose, dict):
        raise ValueError(f'VLAHost 缺少 eef_{side} 当前位姿')
    position = pose.get('position')
    orientation = pose.get('orientation')
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise ValueError(f'VLAHost eef_{side} 位姿格式无效')
    values = [position.get(axis) for axis in ('x', 'y', 'z')]
    values += [orientation.get(axis) for axis in ('x', 'y', 'z', 'w')]
    if any(value is None or not math.isfinite(float(value)) for value in values):
        raise ValueError(f'VLAHost eef_{side} 位姿包含无效数值')
    return [float(value) for value in values]


def pose_in_base(message, transform):
    """把曝光时刻的物体位姿从相机系变换到 base_link。"""
    origin, rotation = transform_pose(transform)
    position = message.pose.position
    translation = (position.x, position.y, position.z)
    orientation = message.pose.orientation
    object_quaternion = normalize(
        (orientation.x, orientation.y, orientation.z, orientation.w))
    if not all(math.isfinite(value) for value in translation):
        raise ValueError('物体位置包含非有限数值')
    offset = rotate_vector(translation, rotation)
    return (tuple(a + b for a, b in zip(origin, offset)),
            normalize(multiply_quaternions(rotation, object_quaternion)))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right'), required=True, help='抓取手臂')
    parser.add_argument('--object-pose-topic', required=True,
                        help='头部视觉输出的 geometry_msgs/PoseStamped 话题')
    parser.add_argument('--pose-request-topic',
                        help='可选：进入采集阶段时向视觉程序发送一次 std_msgs/Empty 请求')
    parser.add_argument('--object-pose-change-topic', default='/object_pose_changed',
                        help=('物体位置变化通知话题，消息类型为 std_msgs/Bool；'
                              '运动阶段收到后中断并按新位置重新采集，空字符串表示禁用'))
    parser.add_argument('--execute', action='store_true', help='预检完成后执行；默认只预览')
    parser.add_argument('--position-only', action='store_true',
                        help='只移动到物体位置；忽略水瓶姿态并保持当前工具姿态')
    parser.add_argument('--reach-only', action='store_true',
                        help='到达抓取点后退出；不执行脚本内的闭合夹爪和抬升')
    parser.add_argument('--approach-only', action='store_true',
                        help='到达预抓取点后退出；不进入抓取点移动、闭爪或抬升阶段')
    parser.add_argument('--control-backend', choices=('joint', 'ros', 'http'), default='joint',
                        help='joint 本地单臂 IK 后发布 user joint source 2；ros 发布 IK；http 使用 VLAHost')
    parser.add_argument('--max-joint-speed-rad-s', type=float, default=0.3,
                        help='joint 模式所选臂每关节的最大命令速度，默认 0.3 rad/s')
    parser.add_argument('--vlahost-url', default='http://6.6.7.100:8011',
                        help='远程 VLAHost HTTP 地址，control-backend=http 时使用')
    parser.add_argument('--http-timeout-s', type=float, default=2.0,
                        help='VLAHost 单次 HTTP 请求超时，单位 s')
    parser.add_argument('--http-action-hz', type=float, default=10.0,
                        help='VLAHost action_chunk 发送频率，范围 1..20 Hz')
    parser.add_argument('--http-log-hz', type=float, default=1.0,
                        help='打印已发送 HTTP action_chunk 的频率，默认 1 Hz；设为 10 可打印每次发送')
    parser.add_argument('--namespace', default='/tj')
    parser.add_argument('--model-xml', help='工具偏移所用 MJCF；默认已安装的 marvin_description')
    parser.add_argument('--ik-topic', default='control/teleop/ik_request')
    parser.add_argument('--ik-source', type=int, default=0)
    parser.add_argument('--joint-source', type=int, default=0)
    parser.add_argument('--capture-timeout', type=float, default=30.0)
    parser.add_argument('--pose-request-interval', type=float, default=1.0,
                        help='未获得稳定物体位姿时的视觉请求重发间隔，单位 s')
    # robot_state_publisher 的动态 TF 缓存启动后通常需要几秒，预留时间避免
    # 物体位姿已经收到但末端 Arm_L7_Link 尚未进入 TF 缓存时误报失败。
    parser.add_argument('--preflight-timeout', type=float, default=15.0)
    parser.add_argument('--motion-timeout', type=float, default=60.0,
                        help='运动超时，单位 s；reach-only 模式从 approach 开始计整个动作')
    parser.add_argument('--feedback-timeout', type=float, default=0.5)
    parser.add_argument('--max-pose-age', type=float, default=0.5,
                        help='物体观测最大年龄，单位 s')
    parser.add_argument('--stable-samples', type=int, default=5)
    parser.add_argument('--stable-position-mm', type=float, default=5.0)
    parser.add_argument('--stable-angle-deg', type=float, default=5.0)
    parser.add_argument('--grasp-offset-mm', nargs=3, type=float, default=(0., 0., 0.),
                        metavar=('X', 'Y', 'Z'), help='base_link 下物体原点到抓取点的偏移')
    parser.add_argument('--intermediate-offset-mm', nargs=3, type=float,
                        metavar=('X', 'Y', 'Z'),
                        help='可选：先到的中间点，相对物体原点的 base_link 偏移；到位后再到抓取点')
    parser.add_argument(
        '--bottle-to-tool-quaternion', nargs=4, type=float,
        default=(-0.5, -0.5, -0.5, 0.5), metavar=('X', 'Y', 'Z', 'W'),
        help=('瓶子坐标系到 left_tool 的固定旋转，ROS xyzw；默认将瓶子前后/左右/竖直轴映射到工具 X/Y/Z'))
    parser.add_argument('--approach-front-mm', type=float, default=0.0,
                        help='预抓取点在抓取点的 base_link -X 方向距离，单位 mm')
    parser.add_argument('--approach-height-mm', type=float, default=30.0)
    parser.add_argument('--lift-height-mm', type=float, default=50.0)
    parser.add_argument('--max-displacement-mm', type=float, default=500.0,
                        help='初始工具位姿到各段目标的最大距离')
    parser.add_argument('--max-target-angle-deg', type=float, default=180.0,
                        help='当前工具姿态到补偿后目标姿态的最大转角')
    parser.add_argument('--position-tolerance-mm', type=float, default=10.0)
    parser.add_argument('--angle-tolerance-deg', type=float, default=2.5)
    parser.add_argument('--approach-position-tolerance-mm', type=float, default=50.0,
                        help='接近阶段稳定到位的位置误差阈值，单位 mm')
    parser.add_argument('--approach-angle-tolerance-deg', type=float, default=5.0,
                        help='接近阶段稳定到位的姿态误差阈值，单位 deg')
    parser.add_argument('--reach-position-tolerance-mm', type=float, default=50.0,
                        help='reach-only 抓取点稳定到位的位置误差阈值，单位 mm')
    parser.add_argument('--reach-angle-tolerance-deg', type=float, default=5.0,
                        help='reach-only 抓取点稳定到位的姿态误差阈值，单位 deg')
    parser.add_argument('--settle-time', type=float, default=1.0)
    parser.add_argument('--gripper-duration', type=float, default=12.0,
                        help='夹爪从完全打开平滑闭合到目标值的时间，单位 s，默认 12 s')
    parser.add_argument('--gripper-rate-hz', type=float, default=10.0,
                        help='夹爪命令发布频率，默认 10 Hz，范围 1..20 Hz')
    args = parser.parse_args(argv)
    positive = ('capture_timeout', 'pose_request_interval', 'preflight_timeout', 'motion_timeout',
                'feedback_timeout', 'max_pose_age', 'stable_position_mm',
                'stable_angle_deg', 'max_displacement_mm', 'max_target_angle_deg',
                'position_tolerance_mm', 'angle_tolerance_deg', 'settle_time',
                'approach_position_tolerance_mm', 'approach_angle_tolerance_deg',
                'reach_position_tolerance_mm', 'reach_angle_tolerance_deg',
                'gripper_duration', 'gripper_rate_hz', 'max_joint_speed_rad_s')
    for name in positive:
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'--{name.replace("_", "-")} 必须为有限正数')
    if args.stable_samples < 2 or not 1.0 <= args.gripper_rate_hz <= 20.0:
        parser.error('--stable-samples 至少为 2；--gripper-rate-hz 范围为 1..20')
    if (not math.isfinite(args.http_timeout_s) or args.http_timeout_s <= 0 or
            not 1.0 <= args.http_action_hz <= 20.0 or
            not math.isfinite(args.http_log_hz) or args.http_log_hz <= 0):
        parser.error('--http-timeout-s 必须为正数；--http-action-hz 范围为 1..20；--http-log-hz 必须为正数')
    for name in ('approach_front_mm', 'approach_height_mm', 'lift_height_mm'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) < 0:
            parser.error(f'--{name.replace("_", "-")} 必须为有限非负数')
    # argparse 对显式传入的 nargs 参数返回 list；统一成 tuple，避免和默认 tuple 混用。
    args.grasp_offset_mm = tuple(args.grasp_offset_mm)
    if args.intermediate_offset_mm is not None:
        args.intermediate_offset_mm = tuple(args.intermediate_offset_mm)
    args.bottle_to_tool_quaternion = tuple(args.bottle_to_tool_quaternion)
    offsets = args.grasp_offset_mm + (args.intermediate_offset_mm or ())
    if not all(math.isfinite(v) for v in offsets + args.bottle_to_tool_quaternion):
        parser.error('抓取偏移必须是有限数值')
    try:
        normalize(args.bottle_to_tool_quaternion)
    except ValueError as error:
        parser.error(str(error))
    return args


class HeadObjectGrasp(Node):
    def __init__(self, args):
        super().__init__('head_object_grasp')
        self.args = args
        self.http_mode = args.control_backend == 'http'
        self.joint_mode = args.control_backend == 'joint'
        self.ik_model = None
        self.ik_data = None
        self.ik_frame_id = None
        if self.joint_mode:
            if pin is None:
                raise ValueError('joint 模式需要 Python pinocchio；请先加载 ROS 环境')
            model_xml = args.model_xml or os.path.join(
                get_package_share_directory('marvin_description'),
                'mjcf', 'Gento_Luna', 'Gento_Luna.xml')
            self.ik_model = pin.buildModelFromMJCF(model_xml)
            if self.ik_model.nq != 22 or self.ik_model.nv != 22:
                raise ValueError(f'MJCF 关节数需为 22，实际 nq={self.ik_model.nq}, nv={self.ik_model.nv}')
            self.ik_data = self.ik_model.createData()
            self.ik_frame_id = self.ik_model.getFrameId(
                'left_tool' if args.side == 'left' else 'right_tool')
            if self.ik_frame_id >= len(self.ik_model.frames):
                raise ValueError('MJCF 缺少所选手臂工具 frame')
        self.http_last_state_at = 0.0
        self.http_state = None
        self.http_session_enabled = False
        self.http_session_released = False
        self.http_generation = None
        self.http_last_session_at = 0.0
        self.http_last_action_at = 0.0
        self.http_last_log_at = 0.0
        # VLAHost 的 EEF 请求要求左右手同时提供目标。未选中手只在第一
        # 次动作前读取一次，后续始终使用该值，避免其目标随反馈漂移。
        self.http_hold_pose = None
        # 中断后重新采集期间发送的 EEF / 关节保持位姿。
        self.interrupt_hold = None
        self.interrupt_joint_hold = None
        self.side = args.side
        self.label = '左' if args.side == 'left' else '右'
        self.index = 0 if args.side == 'left' else 1
        self.arm_link = 'Arm_L7_Link' if self.index == 0 else 'Arm_R7_Link'
        self.tool = 'left_tool' if self.index == 0 else 'right_tool'
        self.mount_position, self.mount_quaternion = read_tool_mount(
            args.model_xml or os.path.join(get_package_share_directory('marvin_description'),
                                            'mjcf', 'Gento_Luna', 'Gento_Luna.xml'),
            'L' if self.index == 0 else 'R')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending_pose = None
        self.pose_change_pending = False
        self.last_pose_stamp = None
        self.samples = deque(maxlen=args.stable_samples)
        self.object_pose = None
        self.object_frozen_at = None
        self.phase = 'capture'
        self.exit_code = None
        self.started = time.monotonic()
        self.phase_started = self.started
        self.motion_started = None
        self.last_log = 0.0
        self.last_preflight_log = 0.0
        self.last_gripper_publish = 0.0
        self.ik_source = None
        self.joint_source = None
        self.states = None
        self.state_at = None
        self.gripper_at = None
        self.gripper_status = None
        self.gripper_status_at = None
        self.target = None
        self.target_quaternion = None
        self.grasp_position = None
        self.intermediate_position = None
        self.approach_position = None
        self.lift_position = None
        self.initial_position = None
        self.settled_since = None
        self.disarm_started = None
        self.joint_positions = None
        self.joint_feedback_at = None
        self.user_joint_hold = None
        self.user_joint_target = None
        self.joint_switch_future = None
        self.joint_switch_started = None
        self.intermediate_reached = False
        self.request_topic = topic_name(args.namespace, args.ik_topic)
        self.gripper_topic = topic_name(args.namespace, f'control/gripperValue{"L" if self.index == 0 else "R"}')
        self.create_subscription(PoseStamped, args.object_pose_topic,
                                 self.on_pose, qos_profile_sensor_data)
        if args.object_pose_change_topic:
            self.create_subscription(Bool, args.object_pose_change_topic,
                                     self.on_pose_change, 10)
        self.pose_request = (self.create_publisher(Empty, args.pose_request_topic,
                                                   qos_profile_sensor_data)
                             if args.pose_request_topic else None)
        self.last_pose_request_at = None
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Int32, topic_name(args.namespace, 'info/ik_request_mux/active_source'),
                                 lambda msg: setattr(self, 'ik_source', msg.data), state_qos)
        self.create_subscription(Int32, topic_name(args.namespace, 'info/joint_cmd_mux/active_source'),
                                 lambda msg: setattr(self, 'joint_source', msg.data), state_qos)
        self.create_subscription(Int16MultiArray, topic_name(args.namespace, 'info/robot_state'),
                                 self.on_state, 10)
        if self.joint_mode:
            self.create_subscription(Jointfeedback, topic_name(args.namespace, 'info/joint_feedback'),
                                     self.on_joint_feedback, qos_profile_sensor_data)
        suffix = 'L' if self.index == 0 else 'R'
        self.create_subscription(Float32MultiArray, topic_name(args.namespace, f'info/gripper_feedback_{suffix}'),
                                 self.on_gripper, 20)
        self.create_subscription(Int32MultiArray, topic_name(args.namespace, f'info/gripper_feedback_{suffix}_err'),
                                 self.on_gripper_status, 20)
        request_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = (None if self.http_mode or self.joint_mode else
                          self.create_publisher(IKRequest, self.request_topic, request_qos))
        self.user_joint_publishers = ()
        self.joint_select_client = None
        if self.joint_mode:
            self.user_joint_publishers = (
                self.create_publisher(JointcmdArm, topic_name(args.namespace, 'control/user/joint_cmd_A'), request_qos),
                self.create_publisher(JointcmdArm, topic_name(args.namespace, 'control/user/joint_cmd_B'), request_qos),
                self.create_publisher(JointcmdBody, topic_name(args.namespace, 'control/user/joint_cmd_body'), request_qos),
                self.create_publisher(JointcmdHead, topic_name(args.namespace, 'control/user/joint_cmd_head'), request_qos),
            )
            self.joint_select_client = self.create_client(
                Int, topic_name(args.namespace, 'control/joint_cmd_mux/select'))
        self.gripper_publisher = (None if self.http_mode else
                                  self.create_publisher(Float32, self.gripper_topic, 10))
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'等待 {args.object_pose_topic} 物体位姿；选用{self.label}臂 {self.tool}；'
                               f'{"执行" if args.execute else "预览"}模式；'
                               f'控制后端={"VLAHost HTTP " + args.vlahost_url if self.http_mode else "user joint source 2" if self.joint_mode else "ROS IK"}')

    def on_pose(self, msg):
        if self.phase == 'capture':
            self.pending_pose = msg

    def on_pose_change(self, msg):
        """记录视觉程序的位置变化通知，由 tick 在运动阶段安全处理。"""
        if msg.data:
            self.pose_change_pending = True

    def on_state(self, msg):
        self.states = tuple(msg.data)
        self.state_at = time.monotonic()

    def on_joint_feedback(self, msg):
        positions = [*msg.body_positions[:6], *msg.head_positions[:2],
                     *msg.arm_positions[:14]]
        if len(positions) == 22 and all(math.isfinite(value) for value in positions):
            self.joint_positions = [float(value) for value in positions]
            self.joint_feedback_at = time.monotonic()

    def on_gripper(self, msg):
        if len(msg.data) >= 3 and all(math.isfinite(v) for v in msg.data[:3]):
            self.gripper_at = time.monotonic()

    def on_gripper_status(self, msg):
        if msg.data:
            self.gripper_status = tuple(msg.data)
            self.gripper_status_at = time.monotonic()

    def capture(self):
        msg = self.pending_pose
        if msg is None:
            return
        self.pending_pose = None
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_pose_stamp:
            return
        self.last_pose_stamp = stamp
        try:
            age = self.get_clock().now().nanoseconds * 1e-9 - (stamp[0] + stamp[1] * 1e-9)
            if stamp == (0, 0) or not -0.05 <= age <= self.args.max_pose_age:
                raise ValueError(f'物体位姿时间戳无效或过旧：{age:.3f} s')
            frame = msg.header.frame_id.strip('/')
            if not frame:
                raise ValueError('物体位姿 frame_id 为空')
            # 相机 TF 由关节反馈时间戳发布，通常会比图像时间落后几十毫秒。
            # 使用最新可用 TF，避免按图像时间查询时触发 future extrapolation。
            transform = self.buffer.lookup_transform(
                'base_link', frame, Time(), timeout=Duration(seconds=0.02))
            position, quaternion = pose_in_base(msg, transform)
            position_changed = (math.dist(position, self.samples[-1][0]) * 1000.0 >
                                self.args.stable_position_mm) if self.samples else False
            orientation_changed = (
                not self.args.position_only and self.samples and
                angular_error_deg(quaternion, self.samples[-1][1]) >
                self.args.stable_angle_deg)
            if position_changed or orientation_changed:
                self.samples.clear()
            self.samples.append((position, quaternion))
            if len(self.samples) == self.args.stable_samples:
                reference = self.samples[-1]
                if all(
                        math.dist(p, reference[0]) * 1000.0 <=
                        self.args.stable_position_mm and (
                            self.args.position_only or
                            angular_error_deg(q, reference[1]) <=
                            self.args.stable_angle_deg)
                        for p, q in self.samples):
                    self.object_pose = reference
                    self.object_frozen_at = time.monotonic()
                    self.phase = 'preflight'
                    self.phase_started = time.monotonic()
                    self.get_logger().info(
                        f'冻结 base_link 物体位置={position}，物体姿态={quaternion}')
        except (ValueError, TransformException) as error:
            self.samples.clear()
            self.get_logger().warning(f'等待有效头部物体位姿 / 相机 TF：{error}',
                                      throttle_duration_sec=1.0)

    def hold_current(self):
        """中断时立即以当前反馈作为保持目标，避免继续执行旧的运动目标。"""
        if self.http_mode:
            try:
                actual, quaternion = self.actual()
            except (ValueError, TransformException) as error:
                self.get_logger().warning(f'中断时无法读取当前 EEF 位姿：{error}')
                return
            self.target, self.target_quaternion = actual, normalize(quaternion)
            self.interrupt_hold = (self.target, self.target_quaternion)
            try:
                self.http_publish(0.0)
            except ValueError as error:
                self.get_logger().warning(f'中断时发送 EEF 保持目标失败：{error}')
            return
        if self.joint_mode and self.user_joint_hold is not None:
            try:
                current = self.feedback_configuration()
            except ValueError as error:
                self.get_logger().warning(f'中断时无法读取关节反馈：{error}')
                return
            self.interrupt_joint_hold = list(current)
            self.user_joint_hold = list(current)
            self.user_joint_target = list(current)
            self.publish_user_joint(self.user_joint_target)
            return
        if self.publisher is not None:
            self.publish_request(False)

    def keep_hold(self):
        """重新采集位姿期间持续保持中断时的位姿，防止旧目标继续执行。"""
        if self.http_mode:
            if self.http_session_enabled and self.interrupt_hold is not None:
                try:
                    self.http_publish(0.0, *self.interrupt_hold)
                except ValueError as error:
                    self.get_logger().warning(
                        f'保持 VLAHost EEF 位姿失败：{error}', throttle_duration_sec=1.0)
            return
        if self.joint_mode and self.interrupt_joint_hold is not None:
            if self.joint_source != USER_JOINT_SOURCE:
                return
            try:
                self.publish_user_joint(self.interrupt_joint_hold)
            except ValueError as error:
                self.get_logger().warning(
                    f'保持关节位姿失败：{error}', throttle_duration_sec=1.0)

    def restart_capture(self, now):
        """丢弃已冻结的物体位姿，回到采集阶段等待新的稳定位置。"""
        self.phase = 'capture'
        self.phase_started = now
        self.started = now
        self.motion_started = None
        self.object_pose = None
        self.object_frozen_at = None
        self.samples.clear()
        self.pending_pose = None
        self.target = None
        self.target_quaternion = None
        self.grasp_position = None
        self.intermediate_position = None
        self.approach_position = None
        self.lift_position = None
        self.initial_position = None
        self.settled_since = None
        self.intermediate_reached = False
        self.last_pose_request_at = None

    def handle_pose_change(self, now):
        """物体被挪动后停止当前目标，并按新位置重新采集和规划。"""
        if self.phase == 'capture':
            # 采集阶段本来就是等待稳定位姿；立即丢弃移动前的样本并重新请求。
            self.samples.clear()
            self.started = now
            self.last_pose_request_at = None
            self.get_logger().info('收到物体位置变化通知，重新开始稳定位姿采集')
            return
        if self.phase in ('closing', 'lift'):
            self.get_logger().warning(
                f'{self.phase} 阶段忽略物体位置变化通知；'
                '闭合夹爪或抬升时物体随机械臂移动')
            return
        self.hold_current()
        self.get_logger().warning(
            f'{self.phase} 阶段收到物体位置变化通知，中断当前目标，'
            '按新位置重新采集和规划')
        self.restart_capture(now)

    def actual(self):
        if getattr(self, 'http_mode', False):
            state = self.http_state_read()
            pose = remote_pose(state, 'left' if self.index == 0 else 'right')
            return tuple(pose[:3]), normalize(pose[3:])
        transform = self.buffer.lookup_transform('base_link', self.arm_link, Time(),
                                                 timeout=Duration(seconds=0.02))
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if stamp <= 0 or not -0.05 <= age <= self.args.feedback_timeout:
            raise ValueError(f'{self.arm_link} TF 已过期：{age:.3f} s')
        position, quaternion = transform_pose(transform)
        offset = rotate_vector(self.mount_position, quaternion)
        return (tuple(a + b for a, b in zip(position, offset)),
                normalize(multiply_quaternions(quaternion, self.mount_quaternion)))

    def http_state_read(self, force=False):
        now = time.monotonic()
        if (not force and self.http_state is not None and
                now - self.http_last_state_at < 1.0 / self.args.http_action_hz):
            return self.http_state
        state = http_json(self.args.vlahost_url, '/state', timeout=self.args.http_timeout_s)
        if not state.get('connected'):
            raise ValueError('VLAHost ROS feedback 尚未连接')
        remote_pose(state, 'left')
        remote_pose(state, 'right')
        self.http_state = state
        self.http_last_state_at = now
        return state

    def http_session(self, enabled):
        result = http_json(
            self.args.vlahost_url, '/intervention/session',
            {'enabled': bool(enabled), 'source': 'head_object_grasp'},
            timeout=self.args.http_timeout_s)
        intervention = result.get('intervention', {})
        if enabled:
            self.http_session_enabled = True
            self.http_generation = intervention.get('refresh_generation')
            self.http_last_session_at = time.monotonic()
        else:
            self.http_session_enabled = False
            self.http_session_released = True
        return result

    def http_keepalive(self):
        if not self.http_session_enabled:
            self.http_session(False)
            self.http_session(True)
        elif time.monotonic() - self.http_last_session_at >= 1.0:
            self.http_session(True)

    def http_publish(self, gripper_value, target=None, quaternion=None):
        now = time.monotonic()
        if now - self.http_last_action_at < 1.0 / self.args.http_action_hz:
            return
        self.http_keepalive()
        state = self.http_state_read(force=True)
        left = remote_pose(state, 'left')
        right = remote_pose(state, 'right')
        if self.http_hold_pose is None:
            self.http_hold_pose = list(right if self.index == 0 else left)
            self.get_logger().info(
                f'HTTP 固定未选中手初始 EEF：'
                f'{"right" if self.index == 0 else "left"}={self.http_hold_pose}')
        target_pose = list(self.target if target is None else target) + list(
            self.target_quaternion if quaternion is None else quaternion)
        if self.index == 0:
            left = target_pose
            right = list(self.http_hold_pose)
        else:
            left = list(self.http_hold_pose)
            right = target_pose
        action = {'eef_left': left, 'eef_right': right}
        if self.index == 0:
            action['gripper_left'] = min(1.0, max(0.0, float(gripper_value)))
        else:
            action['gripper_right'] = min(1.0, max(0.0, float(gripper_value)))
        body = {'actions': [action], 'source_hz': self.args.http_action_hz}
        if self.http_generation is not None:
            body['observation_generation'] = self.http_generation
        result = http_json(self.args.vlahost_url, '/action_chunk', body,
                           timeout=self.args.http_timeout_s)
        if result.get('execution') == 'shadow':
            reason = result.get('reason', 'unknown')
            if reason == 'stale_observation':
                self.http_generation = result.get('required_generation')
            raise ValueError(f'VLAHost action 未执行，execution=shadow，原因={reason}')
        self.http_last_action_at = now
        if (now - self.http_last_log_at >= 1.0 / self.args.http_log_hz or
                self.http_last_log_at == 0.0):
            self.http_last_log_at = now
            payload = json.dumps(body, ensure_ascii=False, separators=(',', ':'))
            response = json.dumps({
                'execution': result.get('execution'),
                'control_space': result.get('control_space'),
                'sequence': result.get('sequence'),
                'safety_clamps': result.get('safety_clamps'),
            }, ensure_ascii=False, separators=(',', ':'))
            self.get_logger().info(f'HTTP→VLA /action_chunk 已发送：{payload}；响应：{response}')

    def release_http(self):
        if self.http_mode and self.http_session_enabled and not self.http_session_released:
            try:
                self.http_session(False)
            except ValueError as error:
                self.get_logger().warning(f'关闭 VLAHost 控制会话失败：{error}')

    def feedback_configuration(self):
        if (self.joint_feedback_at is None or
                time.monotonic() - self.joint_feedback_at > self.args.feedback_timeout or
                self.joint_positions is None):
            raise ValueError('22 关节反馈缺失或过期')
        return list(self.joint_positions)

    def publish_user_joint(self, q):
        if len(q) != 22 or not all(math.isfinite(value) for value in q):
            raise ValueError('user joint 命令必须包含 22 个有限关节角')
        stamp = self.get_clock().now().to_msg()
        left = JointcmdArm(); left.header.stamp = stamp; left.positions = q[8:15]
        right = JointcmdArm(); right.header.stamp = stamp; right.positions = q[15:22]
        body = JointcmdBody(); body.header.stamp = stamp; body.positions = q[:6]
        head = JointcmdHead(); head.header.stamp = stamp; head.positions = [q[6], q[7], 0.0]
        for publisher, message in zip(self.user_joint_publishers, (left, right, body, head)):
            publisher.publish(message)

    def solve_arm_ik(self, q, position, quaternion):
        start = 8 if self.index == 0 else 15
        indices = list(range(start, start + 7))
        position_only = self.args.position_only
        target = pin.SE3(
            pin.Quaternion(float(quaternion[3]), float(quaternion[0]),
                           float(quaternion[1]), float(quaternion[2])).toRotationMatrix(),
            np.asarray(position, dtype=float))
        if position_only:
            if self.phase in ('approach', 'approach_second'):
                angle_limit_deg = self.args.approach_angle_tolerance_deg
                position_limit_mm = self.args.approach_position_tolerance_mm
            elif self.phase == 'descend' and self.args.reach_only:
                angle_limit_deg = self.args.reach_angle_tolerance_deg
                position_limit_mm = self.args.reach_position_tolerance_mm
            else:
                angle_limit_deg = self.args.angle_tolerance_deg
                position_limit_mm = self.args.position_tolerance_mm
            position_limit_mm = min(10.0, position_limit_mm)
        # 优先完整保持姿态；只有在位置模式且 6D 求解失败时才尝试位置 IK。
        # 位置 IK 的结果仍需满足姿态漂移上限，避免工具在接近物体时大幅翻转。
        for use_position_ik in ((False, True) if position_only else (False,)):
            solution = np.asarray(q, dtype=float)
            for _ in range(80):
                pin.forwardKinematics(self.ik_model, self.ik_data, solution)
                pin.updateFramePlacements(self.ik_model, self.ik_data)
                current = self.ik_data.oMf[self.ik_frame_id]
                if use_position_ik:
                    error = target.translation - current.translation
                    reference = pin.LOCAL_WORLD_ALIGNED
                    rows = 3
                else:
                    error = pin.log6(current.inverse() * target).vector
                    reference = pin.LOCAL
                    rows = 6
                if float(np.linalg.norm(error)) < 1e-4:
                    break
                jacobian = pin.computeFrameJacobian(
                    self.ik_model, self.ik_data, solution, self.ik_frame_id, reference)[:rows, indices]
                velocity = jacobian.T @ np.linalg.solve(
                    jacobian @ jacobian.T + 1e-4 * np.eye(rows), error)
                norm = float(np.linalg.norm(velocity))
                if norm > 0.12:
                    velocity *= 0.12 / norm
                delta = np.zeros(self.ik_model.nv)
                delta[indices] = velocity
                solution = pin.integrate(self.ik_model, solution, delta * 0.7)
                for index in indices:
                    solution[index] = min(max(solution[index], self.ik_model.lowerPositionLimit[index]),
                                          self.ik_model.upperPositionLimit[index])
            pin.forwardKinematics(self.ik_model, self.ik_data, solution)
            pin.updateFramePlacements(self.ik_model, self.ik_data)
            final = self.ik_data.oMf[self.ik_frame_id]
            position_error_mm = float(np.linalg.norm(target.translation - final.translation) * 1000.0)
            angle_error_deg = math.degrees(float(np.linalg.norm(
                pin.log3(final.rotation.T @ target.rotation))))
            if position_only:
                converged = (position_error_mm <= position_limit_mm and
                             angle_error_deg <= angle_limit_deg)
            else:
                converged = float(np.linalg.norm(pin.log6(final.inverse() * target).vector)) <= 3e-3
            if converged:
                result = list(q)
                result[start:start + 7] = solution[start:start + 7].tolist()
                return result
        if position_only:
            raise ValueError(
                f'单臂位置 IK 未收敛：位置误差={position_error_mm:.1f} mm'
                f'（上限 {position_limit_mm:g} mm），'
                f'相对保持姿态转角={angle_error_deg:.1f}°（上限 {angle_limit_deg:g}°）；'
                f'目标={self.target}')
        else:
            raise ValueError(
                f'单臂 IK 未收敛：位置误差={position_error_mm:.1f} mm，'
                f'姿态误差={angle_error_deg:.1f}°；目标={self.target}')

    def make_user_joint_target(self, actual, actual_quaternion):
        current = self.feedback_configuration()
        q = list(self.user_joint_hold)
        start = 8 if self.index == 0 else 15
        q[start:start + 7] = current[start:start + 7]
        # 仅将 TF 实测的相对位姿增量施加到当前模型 FK，避免模型绝对偏差造成接管跳动。
        pin.forwardKinematics(self.ik_model, self.ik_data, np.asarray(q))
        pin.updateFramePlacements(self.ik_model, self.ik_data)
        model_pose = self.ik_data.oMf[self.ik_frame_id]
        model_position = model_pose.translation
        model_quaternion = tuple(float(v) for v in pin.Quaternion(model_pose.rotation).coeffs())
        inverse_actual = (-actual_quaternion[0], -actual_quaternion[1],
                          -actual_quaternion[2], actual_quaternion[3])
        delta_quaternion = multiply_quaternions(self.target_quaternion, inverse_actual)
        target_position = tuple(float(model + target - observed) for model, target, observed in
                                zip(model_position, self.target, actual))
        target_quaternion = normalize(multiply_quaternions(delta_quaternion, model_quaternion))
        return self.solve_arm_ik(q, target_position, target_quaternion)

    def activate_user_joint(self, now):
        if self.user_joint_hold is None:
            self.disarm_started = now
        # 切源等待期间继续用最新反馈覆盖旧 user 缓存，避免其它源正在运动时接管旧角度。
        self.user_joint_hold = self.feedback_configuration()
        self.user_joint_target = list(self.user_joint_hold)
        self.publish_user_joint(self.user_joint_hold)
        if now - self.disarm_started < 0.5:
            return False
        if self.joint_source == USER_JOINT_SOURCE:
            return True
        if self.joint_switch_future is None:
            request = Int.Request(); request.data = USER_JOINT_SOURCE
            self.joint_switch_future = self.joint_select_client.call_async(request)
            self.joint_switch_started = now
            return False
        if not self.joint_switch_future.done():
            if now - self.joint_switch_started > 2.0:
                raise ValueError('joint mux 切换 source 2 超时')
            return False
        response = self.joint_switch_future.result()
        if response is None or not response.success:
            raise ValueError(f'joint mux 切换失败：{None if response is None else response.message}')
        if now - self.joint_switch_started > 2.0:
            raise ValueError('joint mux 未确认 source 2')
        return False

    def readiness_problem(self, now):
        if getattr(self, 'http_mode', False):
            try:
                self.http_state_read()
                if not self.http_session_enabled:
                    self.http_session(True)
            except ValueError as error:
                return str(error)
            return None
        if self.joint_mode:
            if self.joint_source is None:
                return '尚未收到 joint mux 当前源'
            if self.phase != 'preflight' and self.joint_source != USER_JOINT_SOURCE:
                return f'joint mux 已切离 source 2，当前 {self.joint_source}'
            if (self.joint_feedback_at is None or
                    now - self.joint_feedback_at > self.args.feedback_timeout or
                    self.joint_positions is None):
                return '22 关节反馈缺失或过期'
            if not self.joint_select_client.service_is_ready():
                return 'joint mux 切源服务不可用'
            for publisher in self.user_joint_publishers:
                if publisher.get_subscription_count() == 0:
                    return f'{publisher.topic_name} 无订阅者'
        elif self.ik_source != self.args.ik_source or self.joint_source != self.args.joint_source:
            return f'控制 mux 源需 IK={self.args.ik_source}, joint={self.args.joint_source}；当前 {self.ik_source}, {self.joint_source}'
        if (self.states is None or self.state_at is None or
                now - self.state_at > self.args.feedback_timeout or len(self.states) < 4 or
                self.states[self.index] not in (1, 2) or self.states[1-self.index] not in (1, 2) or
                self.states[3] not in (1, 2)):
            return f'机器人手臂/躯干状态无效或过期：{self.states}'
        if not self.joint_mode and self.publisher.get_subscription_count() == 0:
            return f'{self.request_topic} 无订阅者'
        # VLAHost 会在同一 IK 话题上保留自己的 publisher。按用户要求忽略它，
        # 不让这个预检警告阻止本地控制流程；mux 源检查仍然照常生效。
        peers = [p for p in self.get_publishers_info_by_topic(self.request_topic)
                 if not (p.node_name == self.get_name() and p.node_namespace == self.get_namespace())
                 and p.node_name != 'luna_vlahost'] if not self.joint_mode else []
        if peers:
            return f'{self.request_topic} 有其他发布者：{[p.node_name for p in peers]}'
        if self.gripper_publisher.get_subscription_count() == 0:
            return f'{self.gripper_topic} 无订阅者'
        if self.gripper_at is None or now - self.gripper_at > self.args.feedback_timeout:
            return f'{self.label}夹爪反馈过期'
        if (self.gripper_status_at is None or
                now - self.gripper_status_at > self.args.feedback_timeout or
                any(value != 1 for value in self.gripper_status)):
            return f'{self.label}夹爪状态异常：{self.gripper_status}'
        return None

    def make_goal(self, actual, quaternion):
        object_position, object_quaternion = self.object_pose
        offset = tuple(v / 1000.0 for v in self.args.grasp_offset_mm)
        grasp = tuple(a + b for a, b in zip(object_position, offset))
        intermediate = None
        if getattr(self.args, 'intermediate_offset_mm', None) is not None:
            intermediate_offset = tuple(v / 1000.0 for v in self.args.intermediate_offset_mm)
            intermediate = tuple(a + b for a, b in zip(object_position, intermediate_offset))
        approach = (grasp[0] - self.args.approach_front_mm / 1000.0,
                    grasp[1], grasp[2] + self.args.approach_height_mm / 1000.0)
        lift = (grasp[0], grasp[1], grasp[2] + self.args.lift_height_mm / 1000.0)
        single_point = self.args.approach_only and approach == grasp
        if self.args.position_only:
            # IKRequest 仍需要姿态字段；保持执行开始时的当前工具姿态，
            # 避免把水瓶姿态转换成夹爪姿态后产生意外翻转。
            goal_quaternion = normalize(quaternion)
        else:
            bottle_to_tool = normalize(self.args.bottle_to_tool_quaternion)
            goal_quaternion = normalize(
                multiply_quaternions(object_quaternion, bottle_to_tool))
        angle = angular_error_deg(quaternion, goal_quaternion)
        if angle > self.args.max_target_angle_deg:
            raise ValueError(
                f'补偿后的目标旋转 {angle:.1f}° 超出 '
                f'{self.args.max_target_angle_deg:g}° 上限')
        checked_targets = (('抓取', grasp),) if single_point else (
            ('抓取', grasp), ('预抓取', approach), ('抬升', lift))
        if intermediate is not None:
            checked_targets = checked_targets + (('中间点', intermediate),)
        for name, position in checked_targets:
            if math.dist(actual, position) * 1000.0 > self.args.max_displacement_mm:
                raise ValueError(f'{name}目标超过 {self.args.max_displacement_mm:g} mm 位移上限')
        self.grasp_position = grasp
        self.intermediate_position = intermediate
        self.approach_position, self.lift_position = approach, lift
        self.target_quaternion = goal_quaternion
        self.initial_position = actual
        mode = '只位置，保持当前工具姿态' if self.args.position_only else '位置+水瓶姿态'
        targets = f'唯一抓取点={grasp}' if single_point else (
            f'抓取={grasp}，预抓取={approach}，抬升={lift}')
        if intermediate is not None:
            targets = f'中间点={intermediate}，第二点={approach}'
        self.get_logger().info(f'{self.tool} 当前={actual}，{targets}，模式={mode}，'
                               f'工具姿态 xyzw={goal_quaternion}，相对当前转角={angle:.1f}°')

    def publish_request(self, available, gripper_value=0.0, actual=None, quaternion=None):
        if getattr(self, 'http_mode', False):
            if available:
                try:
                    self.http_publish(gripper_value)
                except ValueError as error:
                    self.finish(1, f'VLAHost HTTP 控制失败：{error}')
            return
        if self.joint_mode:
            if available:
                if self.joint_source != USER_JOINT_SOURCE:
                    raise ValueError('joint mux 已切离 source 2')
                goal = self.make_user_joint_target(actual, quaternion)
                start = 8 if self.index == 0 else 15
                max_step = self.args.max_joint_speed_rad_s * 0.05
                for index in range(start, start + 7):
                    change = goal[index] - self.user_joint_target[index]
                    goal[index] = self.user_joint_target[index] + max(-max_step, min(change, max_step))
                self.publish_user_joint(goal)
                self.user_joint_target = goal
            return
        request = IKRequest()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = 'base_link'
        request.available = [False] * 11
        if available:
            request.available[self.index] = True
            pose = request.left_hand_pose if self.index == 0 else request.right_hand_pose
            pose.position.x, pose.position.y, pose.position.z = self.target
            (pose.orientation.x, pose.orientation.y, pose.orientation.z,
             pose.orientation.w) = self.target_quaternion
        self.publisher.publish(request)

    def publish_gripper(self, value, now):
        if getattr(self, 'http_mode', False):
            return
        if now - self.last_gripper_publish >= 1.0 / self.args.gripper_rate_hz:
            # 夹爪接口接受 0..1；这里再次限幅，避免浮点误差或后续调用传入越界值。
            value = min(1.0, max(0.0, float(value)))
            self.gripper_publisher.publish(Float32(data=value))
            self.last_gripper_publish = now

    def enter(self, phase, target=None):
        self.phase = phase
        self.phase_started = time.monotonic()
        if phase == 'approach' and self.motion_started is None:
            self.motion_started = self.phase_started
        self.settled_since = None
        if target is not None:
            self.target = target
        self.get_logger().info(f'进入 {phase} 阶段，目标={self.target}')

    def stop_control(self):
        """停止本脚本的控制输出，避免退出后 IK 继续使用最后一个目标。"""
        if getattr(self, 'http_mode', False):
            self.release_http()
        elif self.joint_mode and self.user_joint_hold is not None:
            if self.joint_source == USER_JOINT_SOURCE:
                try:
                    current = self.feedback_configuration()
                    hold = list(self.user_joint_hold)
                    start = 8 if self.index == 0 else 15
                    hold[start:start + 7] = current[start:start + 7]
                    self.publish_user_joint(hold)
                except ValueError as error:
                    self.get_logger().warning(f'退出时无法更新手臂保持目标：{error}')
            self.user_joint_hold = None
            self.user_joint_target = None
        elif getattr(self, 'publisher', None) is not None:
            self.publish_request(False)

    def finish(self, code, reason):
        self.exit_code = code
        self.stop_control()
        if code:
            self.get_logger().error(reason)
        else:
            self.get_logger().info(reason)

    def tick(self):
        if self.exit_code is not None:
            return
        now = time.monotonic()
        # 物体被挪动时先停止当前目标，再按新位置重新采集和规划。
        if getattr(self, 'pose_change_pending', False):
            self.pose_change_pending = False
            self.handle_pose_change(now)
        # reach-only 的两段运动共用一个截止时间，防止进入 descend 后重新计时。
        if (self.args.reach_only and self.motion_started is not None and
                now - self.motion_started >= self.args.motion_timeout):
            self.finish(2, f'抓取点移动总超时 {self.args.motion_timeout:g} s；'
                           f'停在 {self.phase} 阶段，停止后续任务')
            return
        if self.phase == 'capture':
            # 中断后重新采集期间继续发送保持目标，避免旧的运动目标继续执行。
            self.keep_hold()
            request_due = (self.last_pose_request_at is None or
                           now - self.last_pose_request_at >= self.args.pose_request_interval)
            if (self.pose_request is not None and request_due and
                    self.pose_request.get_subscription_count() > 0):
                self.pose_request.publish(Empty())
                self.last_pose_request_at = now
                self.get_logger().info(
                    f'已向 {self.args.pose_request_topic} 请求 F2G-Pose 位姿；'
                    f'未收到稳定位姿前每 {self.args.pose_request_interval:g} s 重发')
            self.capture()
            if self.phase == 'capture' and now - self.started > self.args.capture_timeout:
                self.finish(1, '物体稳定姿态采集超时；检查 PoseStamped 话题、时间戳及相机到 base_link 的 TF')
            return
        try:
            actual, quaternion = self.actual()
        except (ValueError, TransformException) as error:
            if self.phase == 'preflight' and now - self.phase_started < self.args.preflight_timeout:
                self.get_logger().warning(f'等待工具 TF：{error}', throttle_duration_sec=1.0)
                return
            self.finish(1, f'工具 TF 不可用，停止抓取：{error}')
            return
        if self.phase == 'preflight':
            if self.target is None:
                try:
                    self.make_goal(actual, quaternion)
                except ValueError as error:
                    self.finish(1, str(error))
                    return
                self.target = (self.intermediate_position
                               if self.intermediate_position is not None
                               else self.approach_position)
            if not self.args.execute:
                self.finish(0, '预览完成；未发布运动或夹爪命令')
                return
            problem = self.readiness_problem(now)
            if problem:
                if now - self.phase_started > self.args.preflight_timeout:
                    self.finish(1, f'执行预检失败：{problem}')
                elif now - self.last_preflight_log >= 1.0:
                    self.last_preflight_log = now
                    self.get_logger().warning(f'等待执行预检条件：{problem}')
                return
            if self.joint_mode:
                try:
                    if self.activate_user_joint(now):
                        self.enter('approach', self.target)
                except (ValueError, RuntimeError) as error:
                    self.finish(1, f'user joint 接管失败：{error}')
            else:
                if self.disarm_started is None:
                    self.disarm_started = now
                self.publish_request(False)
                if now - self.disarm_started >= 0.5:
                    self.enter('approach', self.target)
            return
        problem = self.readiness_problem(now)
        if problem:
            self.finish(1, f'执行期间预检失败，停止指令：{problem}')
            return
        if self.phase in ('approach', 'approach_second', 'descend', 'lift'):
            if now - self.phase_started > self.args.motion_timeout:
                error_mm = math.dist(actual, self.target) * 1000.0
                angle_deg = angular_error_deg(quaternion, self.target_quaternion)
                self.finish(2, f'{self.phase} 超时，停止发送目标；'
                               f'最终误差 {error_mm:.1f} mm / {angle_deg:.1f}°')
                return
            gripper_value = 0.0 if self.phase != 'lift' else 1.0
            try:
                self.publish_request(True, gripper_value, actual, quaternion)
            except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
                self.finish(1, f'运动目标生成失败：{error}')
                return
            self.publish_gripper(gripper_value, now)
            error_mm = math.dist(actual, self.target) * 1000.0
            angle_deg = angular_error_deg(quaternion, self.target_quaternion)
            if self.phase in ('approach', 'approach_second'):
                position_tolerance_mm = self.args.approach_position_tolerance_mm
                angle_tolerance_deg = self.args.approach_angle_tolerance_deg
            elif self.phase == 'descend' and self.args.reach_only:
                position_tolerance_mm = self.args.reach_position_tolerance_mm
                angle_tolerance_deg = self.args.reach_angle_tolerance_deg
            else:
                position_tolerance_mm = self.args.position_tolerance_mm
                angle_tolerance_deg = self.args.angle_tolerance_deg
            if now - self.last_log >= 1.0:
                self.last_log = now
                delta_mm = tuple((target - current) * 1000.0
                                 for target, current in zip(self.target, actual))
                self.get_logger().info(
                    f'{self.phase} 误差={error_mm:.1f} mm / {angle_deg:.1f}°；'
                    f'目标-实测 xyz=({delta_mm[0]:+.1f}, {delta_mm[1]:+.1f}, '
                    f'{delta_mm[2]:+.1f}) mm；'
                    f'完成阈值={position_tolerance_mm:g} mm / {angle_tolerance_deg:g}°，'
                    f'稳定 {self.args.settle_time:g} s')
            if (error_mm <= position_tolerance_mm and
                    angle_deg <= angle_tolerance_deg):
                if self.settled_since is None:
                    self.settled_since = now
                elif now - self.settled_since >= self.args.settle_time:
                    if self.phase in ('approach', 'approach_second'):
                        if (getattr(self, 'intermediate_position', None) is not None and
                                not self.intermediate_reached):
                            self.intermediate_reached = True
                            self.enter('approach_second', self.approach_position)
                        elif self.args.approach_only:
                            single_point = (getattr(self, 'approach_position', None) is not None and
                                            self.approach_position == self.grasp_position)
                            point_name = '抓取点' if single_point else '预抓取点'
                            suffix = '夹爪保持打开' if single_point else '夹爪保持打开，未移动到抓取点'
                            self.finish(0, f'PASS: {self.label}手到达{point_name}；'
                                           f'最终误差 {error_mm:.1f} mm / {angle_deg:.1f}°；{suffix}')
                        else:
                            self.enter('descend', self.grasp_position)
                    elif self.phase == 'descend':
                        if self.args.reach_only:
                            self.finish(0, f'PASS: {self.label}手进入抓取点容差范围；'
                                           f'最终误差 {error_mm:.1f} mm / {angle_deg:.1f}°；'
                                           '夹爪保持打开，尚未验证抓住水瓶')
                        else:
                            self.enter('closing')
                    else:
                        self.finish(0, f'PASS: {self.label}手完成抓取和抬升；最终误差 {error_mm:.1f} mm')
            else:
                self.settled_since = None
        elif self.phase == 'closing':
            error_mm = math.dist(actual, self.target) * 1000.0
            angle_deg = angular_error_deg(quaternion, self.target_quaternion)
            if error_mm > max(40.0, self.args.position_tolerance_mm) or angle_deg > max(8.0, self.args.angle_tolerance_deg):
                self.finish(2, f'闭合时工具偏离抓取点 {error_mm:.1f} mm / {angle_deg:.1f}°，停止加大夹紧量')
                return
            elapsed = now - self.phase_started
            # 先保持完全打开 0.5 s，再用 smoothstep 缓慢起步、缓慢收尾，
            # 避免从 0 直接跳到较大闭合量造成夹爪冲击。
            progress = smoothstep01(
                (elapsed - 0.5) / self.args.gripper_duration)
            try:
                self.publish_request(True, progress, actual, quaternion)
            except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
                self.finish(1, f'保持目标生成失败：{error}')
                return
            self.publish_gripper(progress, now)
            if elapsed >= self.args.gripper_duration + 2.5:
                if self.args.lift_height_mm:
                    self.enter('lift', self.lift_position)
                else:
                    self.finish(0, f'PASS: {self.label}手完成抓取')


def main():
    args = parse_args()
    rclpy.init(args=[])
    node = None
    try:
        node = HeadObjectGrasp(args)
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.exit_code if node.exit_code is not None else 1
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, TransformException) as error:
        print(f'head_object_grasp: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.stop_control()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
