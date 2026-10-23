#!/usr/bin/python3
# 用途：通过 AprilTag 执行左手抓取、松爪与回位流程；已合并原 left_tool_accuracy_test.py（左手 QP 控制）和 apriltag_left_tool_accuracy_test.py（Tag 识别与目标位姿），不再依赖同目录其他脚本。
# 运行：source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh && python3 tools/scripts/apriltag_left_tool_grasp_test.py --help
"""AprilTag 左手抓放：放置后上移 50 mm，复位底盘后再回 home。"""

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
from marvin_msgs.msg import IKRequest, JointcmdArm, Jointfeedback
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from std_msgs.msg import (
    Float32, Float32MultiArray, Int16MultiArray, Int32, Int32MultiArray,
)
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from luna_handeye_calibration.apriltag import AprilTagDetector
from luna_handeye_calibration.kinematics import Kinematics
from luna_handeye_calibration.se3 import transform_from_dict
from luna_handeye_calibration.track_left_apriltag import (
    StableTagCapture, tag_eef_target, transform_message_to_matrix,
)


# ==========================================================================
# 第 1 部分：左手 QP 控制与精度测试（AccuracyTest）
# 来源：原 left_tool_accuracy_test.py
# 与其它部分重名的函数已改名：
# parse_args -> accuracy_parse_args；
# angular_error_deg -> accuracy_angular_error_deg。
# 后者报错文案为“TF 四元数无效”，与第 2、3 部分的 angular_error_deg（“四元数无效”）
# 不同，因此保留两份，不要合并。
# ==========================================================================


def accuracy_parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        '--xyz', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
        help='base_link 下的绝对目标位置，单位 m；保留当前 left_tool 方向')
    group.add_argument(
        '--offset', nargs=3, type=float, metavar=('DX', 'DY', 'DZ'),
        default=(0.005, 0.0, 0.0),
        help='相对当前位置的位移，单位 m，默认沿 base_link +X 走 5 mm')
    parser.add_argument(
        '--offset-frame', choices=('base_link', 'left_tool'),
        default='base_link', help='--offset 使用的坐标轴')
    parser.add_argument(
        '--execute', action='store_true', help='通过 IK mux 发布目标')
    parser.add_argument(
        '--timeout', type=float, default=180.0,
        help='到达最终目标的总超时，单位 s')
    parser.add_argument('--position-tolerance-mm', type=float, default=1.0)
    parser.add_argument('--angle-tolerance-deg', type=float, default=0.5)
    parser.add_argument(
        '--settle-time', type=float, default=0.5,
        help='连续到位时间，单位 s')
    parser.add_argument(
        '--max-displacement-mm', type=float, default=500.0,
        help='从起点到最终目标的总位移上限，默认 500 mm')
    parser.add_argument('--feedback-timeout', type=float, default=0.5)
    parser.add_argument('--preflight-timeout', type=float, default=5.0)
    parser.add_argument(
        '--disarm-time', type=float, default=0.5,
        help='执行前持续发送空请求的时间，满足 teleop 源重新使能握手，单位 s')
    parser.add_argument(
        '--namespace', default='/tj',
        help='QP 与 mux 的 ROS 命名空间，默认 /tj')
    parser.add_argument(
        '--ik-topic', default='control/teleop/ik_request',
        help='IK 请求输入话题，默认使用 mux 源 0 的 teleop 话题')
    parser.add_argument(
        '--qp-pose-topic', default='info/upper_body_ik_result',
        help='QP 内部上半身位姿话题，默认 info/upper_body_ik_result')
    parser.add_argument(
        '--qp-left-hand-index', type=int, default=5,
        help='PoseArray 中 left_hand 的索引，当前 QP 默认是 5')
    parser.add_argument(
        '--qp-joint-topic', default='control/qp_controller/joint_cmd_A',
        help='QP 左臂关节命令话题')
    parser.add_argument(
        '--mux-joint-topic', default='control/joint_cmd_A',
        help='joint_cmd_mux 输出的左臂关节命令话题')
    parser.add_argument(
        '--joint-feedback-topic', default='info/joint_feedback',
        help='实机关节反馈话题')
    parser.add_argument(
        '--model-xml',
        help='Luna MJCF 文件路径，默认读取已安装的 marvin_description')
    parser.add_argument('--ik-source', type=int, default=0)
    parser.add_argument('--joint-source', type=int, default=0)
    args = parser.parse_args()
    values = args.xyz if args.xyz is not None else args.offset
    if not all(math.isfinite(value) for value in values):
        parser.error('目标坐标必须是有限数值')
    if args.qp_left_hand_index < 0:
        parser.error('qp_left_hand_index 必须大于等于 0')
    for name in ('timeout', 'position_tolerance_mm',
                 'angle_tolerance_deg', 'settle_time', 'max_displacement_mm',
                 'feedback_timeout', 'preflight_timeout', 'disarm_time'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f'{name} 必须是有限正数')
    return args


def topic_name(namespace, name):
    """将控制话题解析到指定命名空间；绝对话题保持原样。"""
    if name.startswith('/'):
        return name
    prefix = namespace.strip('/')
    return '/' + '/'.join(part for part in (prefix, name) if part)


def rotate_vector(vector, quaternion):
    """将工具坐标系向量旋转到 base_link 坐标系。"""
    x, y, z, w = quaternion
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-12 or not math.isfinite(norm):
        raise ValueError('TF 四元数无效')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    vx, vy, vz = vector
    # 使用 q * v * q^-1；不修改原始 TF 姿态。
    tx = 2.0 * (y*vz - z*vy)
    ty = 2.0 * (z*vx - x*vz)
    tz = 2.0 * (x*vy - y*vx)
    return (vx + w*tx + y*tz - z*ty,
            vy + w*ty + z*tx - x*tz,
            vz + w*tz + x*ty - y*tx)


def multiply_quaternions(first, second):
    """按 ROS xyzw 顺序计算 first * second。"""
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)


def read_tool_mount(model_xml, side):
    """从 QP 所用 Luna MJCF 读取 Arm_L7/R7_Link 到对应 tool 的固定变换。"""
    if side not in ('L', 'R'):
        raise ValueError(f'无效手臂侧别：{side}')
    tool_name = 'left_tool' if side == 'L' else 'right_tool'
    arm_name = f'Arm_{side}7_Link'
    flange_name = f'flange_{side}'
    root = ET.parse(model_xml).getroot()
    arm = root.find(f".//body[@name='{arm_name}']")
    flange = None if arm is None else arm.find(
        f"body[@name='{flange_name}']")
    tool = None if flange is None else flange.find(
        f"body[@name='{tool_name}']")
    if tool is None:
        raise ValueError(
            f'MJCF 中找不到 {arm_name}/{flange_name}/{tool_name}')

    def placement(body):
        position = tuple(float(v) for v in body.get('pos', '0 0 0').split())
        wxyz = tuple(float(v) for v in body.get('quat', '1 0 0 0').split())
        if (len(position) != 3 or len(wxyz) != 4 or
                not all(math.isfinite(v) for v in position + wxyz)):
            raise ValueError('MJCF 工具固定变换无效')
        return position, (wxyz[1], wxyz[2], wxyz[3], wxyz[0])

    flange_position, flange_quaternion = placement(flange)
    tool_position, tool_quaternion = placement(tool)
    rotated = rotate_vector(tool_position, flange_quaternion)
    position = tuple(a+b for a, b in zip(flange_position, rotated))
    quaternion = multiply_quaternions(flange_quaternion, tool_quaternion)
    return position, quaternion


def accuracy_angular_error_deg(actual, target):
    dot = abs(sum(a*b for a, b in zip(actual, target)))
    norm_a = math.sqrt(sum(value*value for value in actual))
    norm_b = math.sqrt(sum(value*value for value in target))
    if norm_a < 1e-12 or norm_b < 1e-12:
        raise ValueError('TF 四元数无效')
    return math.degrees(2.0 * math.acos(min(1.0, dot/(norm_a*norm_b))))


def transform_pose(stamped):
    p = stamped.transform.translation
    q = stamped.transform.rotation
    position = (p.x, p.y, p.z)
    quaternion = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(value) for value in position + quaternion):
        raise ValueError('TF 包含非有限数值')
    return position, quaternion


class AccuracyTest(Node):
    def __init__(self, args):
        super().__init__('left_tool_accuracy_test')
        self.args = args
        self.ik_topic = topic_name(args.namespace, args.ik_topic)
        self.qp_pose_topic = topic_name(
            args.namespace, args.qp_pose_topic)
        self.qp_joint_topic = topic_name(
            args.namespace, args.qp_joint_topic)
        self.mux_joint_topic = topic_name(
            args.namespace, args.mux_joint_topic)
        self.joint_feedback_topic = topic_name(
            args.namespace, args.joint_feedback_topic)
        model_xml = args.model_xml
        if model_xml is None:
            model_xml = os.path.join(
                get_package_share_directory('marvin_description'),
                'mjcf', 'Gento_Luna', 'Gento_Luna.xml')
        self.left_mount_position, self.left_mount_quaternion = (
            read_tool_mount(model_xml, 'L'))
        self.right_mount_position, self.right_mount_quaternion = (
            read_tool_mount(model_xml, 'R'))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.ik_source = None
        self.joint_source = None
        self.start = time.monotonic()
        self.motion_start = None
        self.disarm_start = None
        self.disarm_done = False
        self.settled_since = None
        self.last_log = 0.0
        self.target = None
        self.target_quaternion = None
        self.start_left_position = None
        self.start_left_quaternion = None
        self.start_right_position = None
        self.last_error = None
        self.last_valid_tf = None
        self.qp_left_pose = None
        self.qp_pose_received_at = None
        self.qp_pose_problem = '尚未收到 QP 内部位姿'
        self.qp_pose_problem_logged = False
        self.qp_joint_cmd = None
        self.qp_joint_cmd_received_at = None
        self.mux_joint_cmd = None
        self.mux_joint_cmd_received_at = None
        self.left_joint_feedback = None
        self.left_joint_feedback_received_at = None
        self.exit_code = None

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Int32,
            topic_name(args.namespace, 'info/ik_request_mux/active_source'),
            self.on_ik_source, state_qos)
        self.create_subscription(
            Int32,
            topic_name(args.namespace, 'info/joint_cmd_mux/active_source'),
            self.on_joint_source, state_qos)
        request_qos = QoSProfile(depth=10)
        request_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            PoseArray, self.qp_pose_topic,
            self.on_qp_pose, request_qos)
        self.create_subscription(
            JointcmdArm, self.qp_joint_topic,
            self.on_qp_joint_cmd, request_qos)
        self.create_subscription(
            JointcmdArm, self.mux_joint_topic,
            self.on_mux_joint_cmd, request_qos)
        self.create_subscription(
            Jointfeedback, self.joint_feedback_topic,
            self.on_joint_feedback, request_qos)
        self.publisher = self.create_publisher(
            IKRequest, self.ik_topic, request_qos)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            '等待真实机器人 base_link -> Arm_L7_Link/Arm_R7_Link TF；工具偏移来自 '
            + model_xml)
        self.get_logger().info(f'IK 请求话题：{self.ik_topic}')
        self.get_logger().info(
            f'QP 内部 left_hand：{self.qp_pose_topic} '
            f'poses[{self.args.qp_left_hand_index}]')
        self.get_logger().info(
            '左臂关节诊断：QP=%s, mux=%s, feedback=%s' %
            (self.qp_joint_topic, self.mux_joint_topic,
             self.joint_feedback_topic))

    def on_ik_source(self, message):
        self.ik_source = message.data

    def on_joint_source(self, message):
        self.joint_source = message.data

    def on_qp_pose(self, message):
        """缓存 QP 内部模型根据命令状态计算出的 left_hand 位姿。"""
        index = self.args.qp_left_hand_index
        if len(message.poses) <= index:
            self.qp_pose_problem = (
                f'PoseArray 只有 {len(message.poses)} 项，无法读取索引 {index}')
            if not self.qp_pose_problem_logged:
                self.get_logger().warning(self.qp_pose_problem)
                self.qp_pose_problem_logged = True
            return
        pose = message.poses[index]
        position = (pose.position.x, pose.position.y, pose.position.z)
        quaternion = (pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w)
        if not all(math.isfinite(value)
                   for value in position + quaternion):
            self.qp_pose_problem = 'QP 内部 left_hand 位姿包含非有限数值'
            if not self.qp_pose_problem_logged:
                self.get_logger().warning(self.qp_pose_problem)
                self.qp_pose_problem_logged = True
            return
        quaternion_norm = math.sqrt(sum(value*value for value in quaternion))
        if quaternion_norm < 1e-12:
            self.qp_pose_problem = 'QP 内部 left_hand 四元数无效'
            if not self.qp_pose_problem_logged:
                self.get_logger().warning(self.qp_pose_problem)
                self.qp_pose_problem_logged = True
            return
        self.qp_left_pose = (
            position, quaternion, message.header.frame_id.strip('/'))
        self.qp_pose_received_at = time.monotonic()
        self.qp_pose_problem = None
        self.qp_pose_problem_logged = False

    @staticmethod
    def valid_joint_values(values):
        return len(values) == 7 and all(
            math.isfinite(value) for value in values)

    def on_qp_joint_cmd(self, message):
        values = tuple(float(value) for value in message.positions)
        if self.valid_joint_values(values):
            self.qp_joint_cmd = values
            self.qp_joint_cmd_received_at = time.monotonic()

    def on_mux_joint_cmd(self, message):
        values = tuple(float(value) for value in message.positions)
        if self.valid_joint_values(values):
            self.mux_joint_cmd = values
            self.mux_joint_cmd_received_at = time.monotonic()

    def on_joint_feedback(self, message):
        values = tuple(float(value) for value in message.arm_positions[:7])
        if self.valid_joint_values(values):
            self.left_joint_feedback = values
            self.left_joint_feedback_received_at = time.monotonic()

    def get_qp_left_in_base(self, now):
        """将最新 QP 内部 left_hand 位姿转换到 base_link。"""
        if self.qp_left_pose is None or self.qp_pose_received_at is None:
            raise ValueError(self.qp_pose_problem or '尚未收到 QP 内部位姿')
        age = now - self.qp_pose_received_at
        if age > self.args.feedback_timeout:
            raise ValueError(f'QP 内部位姿已过期：{age:.3f} s')
        position, quaternion, frame = self.qp_left_pose
        if not frame:
            raise ValueError('QP 内部位姿 frame_id 为空')
        if frame == 'base_link':
            return position, quaternion

        stamped = self.buffer.lookup_transform(
            'base_link', frame, Time(),
            timeout=Duration(seconds=0.02))
        frame_position, frame_quaternion = transform_pose(stamped)
        rotated = rotate_vector(position, frame_quaternion)
        base_position = tuple(
            origin+offset for origin, offset in zip(frame_position, rotated))
        base_quaternion = multiply_quaternions(
            frame_quaternion, quaternion)
        return base_position, base_quaternion

    def qp_diagnostic_text(self, actual, quaternion, now):
        """生成 QP 内部模型与实机 TF 的同帧误差诊断。"""
        try:
            qp_position, qp_quaternion = self.get_qp_left_in_base(now)
        except (TransformException, ValueError) as error:
            return f'qp_internal=unavailable({error})'
        qp_target_mm = math.dist(qp_position, self.target) * 1000.0
        qp_target_angle_deg = accuracy_angular_error_deg(
            qp_quaternion, self.target_quaternion)
        qp_minus_tf_mm = tuple(
            (qp-robot) * 1000.0
            for qp, robot in zip(qp_position, actual))
        qp_tf_distance_mm = math.dist(qp_position, actual) * 1000.0
        qp_tf_angle_deg = accuracy_angular_error_deg(qp_quaternion, quaternion)
        return (
            'qp_internal_xyz=[%.6f, %.6f, %.6f] '
            'qp_target_error=%.3f mm qp_target_angle=%.3f deg '
            'qp_minus_tf_mm=[%.3f, %.3f, %.3f] '
            'qp_tf_distance=%.3f mm qp_tf_angle=%.3f deg' %
            (*qp_position, qp_target_mm, qp_target_angle_deg,
             *qp_minus_tf_mm, qp_tf_distance_mm, qp_tf_angle_deg))

    def fresh_joint_sample(self, values, received_at, name, now):
        if values is None or received_at is None:
            raise ValueError(f'尚未收到 {name}')
        age = now - received_at
        if age > self.args.feedback_timeout:
            raise ValueError(f'{name} 已过期：{age:.3f} s')
        return values, age

    @staticmethod
    def joint_difference_deg(first, second):
        differences = tuple(
            math.degrees(a-b) for a, b in zip(first, second))
        rms = math.sqrt(
            sum(value*value for value in differences) / len(differences))
        worst_index = max(
            range(len(differences)), key=lambda index: abs(differences[index]))
        return differences, rms, worst_index

    @staticmethod
    def format_joint_vector(values, digits=5):
        return '[' + ', '.join(f'{value:.{digits}f}' for value in values) + ']'

    def joint_diagnostic_lines(self, now):
        """比较 QP输出、mux最终输出和左臂实机关节反馈。"""
        try:
            qp, qp_age = self.fresh_joint_sample(
                self.qp_joint_cmd, self.qp_joint_cmd_received_at,
                'QP左臂命令', now)
            mux, mux_age = self.fresh_joint_sample(
                self.mux_joint_cmd, self.mux_joint_cmd_received_at,
                'mux左臂命令', now)
            feedback, feedback_age = self.fresh_joint_sample(
                self.left_joint_feedback,
                self.left_joint_feedback_received_at,
                '左臂关节反馈', now)
        except ValueError as error:
            return [f'JOINT_L unavailable({error})']

        qp_mux, qp_mux_rms, qp_mux_worst = self.joint_difference_deg(
            qp, mux)
        mux_feedback, mux_feedback_rms, mux_feedback_worst = (
            self.joint_difference_deg(mux, feedback))
        qp_feedback, qp_feedback_rms, qp_feedback_worst = (
            self.joint_difference_deg(qp, feedback))
        values_line = (
            'JOINT_L rad qp=%s mux=%s feedback=%s age_ms=[%.1f, %.1f, %.1f]' %
            (self.format_joint_vector(qp), self.format_joint_vector(mux),
             self.format_joint_vector(feedback), qp_age*1000.0,
             mux_age*1000.0, feedback_age*1000.0))
        difference_line = (
            'JOINT_L diff_deg qp-mux=%s rms=%.3f max=J%d:%.3f | '
            'mux-feedback=%s rms=%.3f max=J%d:%.3f | '
            'qp-feedback=%s rms=%.3f max=J%d:%.3f' %
            (self.format_joint_vector(qp_mux, 3), qp_mux_rms,
             qp_mux_worst+1, qp_mux[qp_mux_worst],
             self.format_joint_vector(mux_feedback, 3), mux_feedback_rms,
             mux_feedback_worst+1, mux_feedback[mux_feedback_worst],
             self.format_joint_vector(qp_feedback, 3), qp_feedback_rms,
             qp_feedback_worst+1, qp_feedback[qp_feedback_worst]))
        return [values_line, difference_line]

    def get_tool_actual(self, link_name, mount_position, mount_quaternion):
        stamped = self.buffer.lookup_transform(
            'base_link', link_name, Time(),
            timeout=Duration(seconds=0.02))
        stamp = stamped.header.stamp.sec + stamped.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if stamp <= 0.0 or not -0.05 <= age <= self.args.feedback_timeout:
            raise ValueError(
                f'{link_name} TF 时间戳无效或过旧：{age:.3f} s')
        arm_position, arm_quaternion = transform_pose(stamped)
        rotated = rotate_vector(mount_position, arm_quaternion)
        position = tuple(a+b for a, b in zip(arm_position, rotated))
        quaternion = multiply_quaternions(
            arm_quaternion, mount_quaternion)
        return position, quaternion

    def get_actuals(self):
        left = self.get_tool_actual(
            'Arm_L7_Link', self.left_mount_position,
            self.left_mount_quaternion)
        right = self.get_tool_actual(
            'Arm_R7_Link', self.right_mount_position,
            self.right_mount_quaternion)
        return left, right

    def check_mux(self):
        if self.ik_source != self.args.ik_source:
            return f'IK mux 源为 {self.ik_source}，需要 {self.args.ik_source}'
        if self.joint_source != self.args.joint_source:
            return (f'joint command mux 源为 {self.joint_source}，'
                    f'需要 {self.args.joint_source}')
        if self.publisher.get_subscription_count() == 0:
            return f'{self.ik_topic} 没有订阅者'
        if self.count_publishers(self.ik_topic) > 1:
            return (f'{self.ik_topic} 存在其他发布者；停止 teleop 发布节点后再测试，'
                    '避免两个目标源竞争')
        return None

    def make_target(self, actual, quaternion, right_actual):
        if self.args.xyz is not None:
            goal = tuple(self.args.xyz)
        else:
            offset = tuple(self.args.offset)
            if self.args.offset_frame == 'left_tool':
                offset = rotate_vector(offset, quaternion)
            goal = tuple(a+b for a, b in zip(actual, offset))
        displacement = math.dist(actual, goal)
        if displacement * 1000.0 > self.args.max_displacement_mm + 1e-9:
            raise ValueError(
                f'目标距当前位置 {displacement*1000.0:.3f} mm，超过 '
                f'{self.args.max_displacement_mm:.3f} mm')
        self.target = goal
        self.target_quaternion = quaternion
        self.start_left_position = actual
        self.start_left_quaternion = quaternion
        self.start_right_position = right_actual
        self.get_logger().info(
            'CURRENT base_link->left_tool xyz=[%.6f, %.6f, %.6f]' % actual)
        self.get_logger().info(
            'CURRENT base_link->right_tool xyz=[%.6f, %.6f, %.6f]' %
            right_actual)
        self.get_logger().info(
            'TARGET  base_link->left_tool xyz=[%.6f, %.6f, %.6f], '
            'displacement=%.3f mm' % (*goal, displacement*1000.0))
        if displacement > 0.05:
            self.get_logger().warning(
                '单次目标超过 50 mm；未验证沿途碰撞，执行前须检查整条路径')

    def publish_target(self):
        message = IKRequest()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.available = [True] + [False] * 10
        position = message.left_hand_pose.position
        orientation = message.left_hand_pose.orientation
        position.x, position.y, position.z = self.target
        orientation.x, orientation.y, orientation.z, orientation.w = (
            self.target_quaternion)
        self.publisher.publish(message)

    def publish_disarm(self):
        """发送全 unavailable 请求，让 teleop 源完成 go-home 后的重新使能握手。"""
        message = IKRequest()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.available = [False] * 11
        self.publisher.publish(message)

    def finish(self, code, reason):
        self.exit_code = code
        if code == 0:
            self.get_logger().info(reason)
        else:
            self.get_logger().error(reason)

    def use_absolute_error_settle(self):
        """是否允许用目标绝对位置/姿态误差结束运动。

        普通精度测试需要该判定；夹爪脚本改用连续窗口内的实际位姿抖动，
        因此会覆盖此方法关闭旧的绝对误差触发路径。
        """
        return True

    def tick(self):
        if self.exit_code is not None:
            return
        now = time.monotonic()
        try:
            (actual, quaternion), (right_actual, _) = self.get_actuals()
        except (TransformException, ValueError) as error:
            if now - self.last_log >= 1.0:
                self.last_log = now
                self.get_logger().warning(f'等待有效 TF：{error}')
            if (self.motion_start is None and
                    now - self.start > self.args.preflight_timeout):
                self.finish(
                    1, '预检超时：未收到有效 Arm_L7_Link/Arm_R7_Link TF')
            elif (self.motion_start is not None and
                  self.last_valid_tf is not None and
                  now - self.last_valid_tf > self.args.feedback_timeout):
                self.finish(1, '运动期间 TF 失效，停止发布目标')
            return
        self.last_valid_tf = now

        if self.target is None:
            if self.args.execute:
                problem = self.check_mux()
                if problem:
                    if now - self.start > self.args.preflight_timeout:
                        self.finish(1, '预检失败：' + problem)
                    return
                if not self.disarm_done:
                    if self.disarm_start is None:
                        self.disarm_start = now
                        self.get_logger().info('发送空请求，准备 teleop 源重新使能')
                    self.publish_disarm()
                    if now - self.disarm_start < self.args.disarm_time:
                        return
                    self.disarm_done = True
            try:
                self.make_target(actual, quaternion, right_actual)
            except ValueError as error:
                self.finish(1, str(error))
                return
            if not self.args.execute:
                self.finish(0, '预览完成；未发布运动指令')
                return
            self.motion_start = now
            self.get_logger().warning('开始持续发送最终目标；请保持急停可用')

        problem = self.check_mux()
        if problem:
            self.finish(1, '控制源变化，停止发布目标：' + problem)
            return
        if now - self.motion_start >= self.args.timeout:
            self.finish(2, 'FAIL: 超时；距目标 %.3f mm' %
                        (math.dist(actual, self.target)*1000.0))
            return
        position_mm = math.dist(actual, self.target) * 1000.0
        self.publish_target()
        angle_deg = accuracy_angular_error_deg(quaternion, self.target_quaternion)
        left_delta_mm = math.dist(
            actual, self.start_left_position) * 1000.0
        right_delta_mm = math.dist(
            right_actual, self.start_right_position) * 1000.0
        self.last_error = (position_mm, angle_deg)
        if now - self.last_log >= 1.0:
            self.last_log = now
            qp_diagnostic = self.qp_diagnostic_text(
                actual, quaternion, now)
            self.get_logger().info(
                'left_tool_xyz=[%.6f, %.6f, %.6f] '
                'left_delta=%.3f mm target_error=%.3f mm '
                'angle_error=%.3f deg | '
                'right_tool_xyz=[%.6f, %.6f, %.6f] '
                'right_delta=%.3f mm | %s' %
                (*actual, left_delta_mm, position_mm, angle_deg,
                 *right_actual, right_delta_mm, qp_diagnostic))
            for diagnostic_line in self.joint_diagnostic_lines(now):
                self.get_logger().info(diagnostic_line)
        if not self.use_absolute_error_settle():
            return

        within = (position_mm <= self.args.position_tolerance_mm and
                  angle_deg <= self.args.angle_tolerance_deg)
        if within:
            if self.settled_since is None:
                self.settled_since = now
            elif now - self.settled_since >= self.args.settle_time:
                self.finish(0, 'PASS: left_tool 连续到达目标；最终误差 '
                            f'{position_mm:.3f} mm / {angle_deg:.3f} deg')
                return
        else:
            self.settled_since = None


# ==========================================================================
# 第 2 部分：AprilTag 识别与目标位姿（TagCapture / TagAccuracy）
# 来源：原 apriltag_left_tool_accuracy_test.py
# parse_args -> tag_parse_args；本部分的 angular_error_deg 同时供第 3 部分使用。
# ==========================================================================


WORKSPACE = Path('/home/liuqian/WBC/Apex_Deploy')


def angular_error_deg(actual, target):
    norm_a = math.sqrt(sum(value * value for value in actual))
    norm_b = math.sqrt(sum(value * value for value in target))
    if norm_a < 1e-12 or norm_b < 1e-12:
        raise ValueError('四元数无效')
    dot = min(1.0, abs(sum(a*b for a, b in zip(actual, target))) / (norm_a * norm_b))
    return math.degrees(2.0 * math.acos(dot))


def read_yaml(path):
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


class TagCapture(Node):
    def __init__(self, args):
        super().__init__('apriltag_accuracy_capture')
        self.args = args
        calibration = read_yaml(args.calibration_file)
        if calibration.get('validation_passed') is not True or calibration.get('schema_version') != 1:
            raise ValueError('外参未通过验证')
        if calibration['base_frame'] != 'base_link' or calibration['mount_frame'] != 'Arm_L7_Link':
            raise ValueError('需要 base_link / Arm_L7_Link 的左腕标定')
        self.model = Kinematics(calibration['model_path'], 'base_link', 'Arm_L7_Link')
        # if self.model.sha256 != calibration['model_sha256']:
        #     raise ValueError('标定模型哈希不匹配，请选择已重新验证的外参文件')
        self.mount_camera = transform_from_dict(calibration['T_mount_camera'])
        config = read_yaml(args.offset_config)['/**']['ros__parameters']
        if config.get('tool_frame') != 'left_tool':
            raise ValueError('偏移配置必须针对 left_tool')
        self.offset = config['tag_eef_offset_m']
        self.rpy = config['tag_eef_rpy_deg']
        for value in (self.offset, self.rpy):
            if np.asarray(value).shape != (3,) or not np.isfinite(value).all():
                raise ValueError('Tag→left_tool 偏移和欧拉角必须是三个有限数值')
        tag_eef_target(np.eye(4), self.offset, self.rpy)
        intr = read_yaml(args.intrinsics_file)
        self.size = (intr['image_width'], intr['image_height'])
        self.frame = intr['optical_frame']
        self.K = np.asarray(intr['camera_matrix']['data']).reshape(3, 3)
        self.D = np.asarray(intr['distortion_coefficients']['data'])
        if not np.isfinite(self.K).all() or not np.isfinite(self.D).all():
            raise ValueError('内参包含非有限数值')
        self.distortion = intr['distortion_model']
        self.detector = AprilTagDetector(args.tag_id, args.tag_size, 1.5, 1.5)
        self.capture = StableTagCapture(10, 0.003, 2.0)
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.images = deque(maxlen=30)
        self.processed = None
        self.goal = None
        self.create_subscription(Image, args.image_topic, self.on_image, qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'检测 ID={args.tag_id}，边长={args.tag_size}m；10帧稳定后冻结目标')
        if args.tag_position_only:
            self.get_logger().info('位置测试模式：目标直接使用 Tag 中心，left_tool 保持当前姿态')
        else:
            self.get_logger().info(f'Tag→left_tool 偏移(m)={self.offset}，rpy(deg)={self.rpy}')

    def on_image(self, msg):
        self.images.append((time.monotonic(), msg))

    def tick(self):
        if self.goal is not None:
            return
        # 单线程回调中阻塞等待 TF 会妨碍 TF 自身接收；延迟处理图像让 TF 先到齐。
        ready = [msg for arrival, msg in self.images if time.monotonic() - arrival >= 0.1]
        if not ready:
            return
        msg = ready[-1]
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.processed:
            return
        try:
            age = self.get_clock().now().nanoseconds / 1e9 - (stamp[0] + stamp[1] / 1e9)
            if stamp == (0, 0) or not -0.05 <= age <= 0.5:
                raise ValueError('图像时间戳过旧或无效')
            if (msg.width, msg.height) != self.size or msg.header.frame_id != self.frame:
                raise ValueError('图像分辨率或 optical frame 与内参不匹配')
            # 使用曝光时刻的实机安装连杆 TF，避免移动相机的新旧位姿混用。
            mount = self.tf.lookup_transform('base_link', 'Arm_L7_Link', Time.from_msg(msg.header.stamp),
                                             timeout=Duration(seconds=0.02))
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            camera_tag, reprojection, _ = self.detector.detect(gray, self.K, self.D, self.distortion)
            self.processed = stamp
            base_tag = transform_message_to_matrix(mount.transform) @ self.mount_camera @ camera_tag
            frozen = self.capture.add(base_tag, stamp)
            self.get_logger().info(f'稳定采样 {len(self.capture.samples)}/10，重投影误差={reprojection:.3f}px',
                                   throttle_duration_sec=1.0)
            if frozen is not None:
                self.goal = (frozen.copy() if self.args.tag_position_only else
                             tag_eef_target(frozen, self.offset, self.rpy))
                self.get_logger().info(f'ID {self.args.tag_id} base_link xyz={frozen[:3, 3].tolist()}')
                if self.args.tag_position_only:
                    self.get_logger().info(
                        f'冻结 left_tool 位置={self.goal[:3, 3].tolist()}；姿态将在执行前取当前值')
                else:
                    self.get_logger().info(f'冻结 left_tool xyz={self.goal[:3, 3].tolist()}')
        except (ValueError, TransformException) as exc:
            self.capture.reset()
            self.get_logger().warning(str(exc), throttle_duration_sec=1.0)


class TagAccuracy(AccuracyTest):
    def __init__(self, args, goal):
        self.position_only = args.tag_position_only
        self.goal_quaternion = (None if self.position_only else
                                tuple(Rotation.from_matrix(goal[:3, :3]).as_quat()))
        self.max_angle = args.max_target_angle_deg
        self.robot_ready = False
        self.state_time = 0.0
        self.setpoint_validated = False
        super().__init__(args)
        self.create_subscription(Int16MultiArray, topic_name(args.namespace, 'info/robot_state'),
                                 self.on_state, 10)

    def on_state(self, msg):
        self.robot_ready = len(msg.data) >= 4 and all(msg.data[i] in (1, 2) for i in (0, 1, 3))
        self.state_time = time.monotonic()

    def check_mux(self):
        problem = super().check_mux()
        if problem:
            return problem
        if not self.robot_ready or time.monotonic() - self.state_time > self.args.feedback_timeout:
            return '等待新鲜的左右臂和躯干可运动状态'
        # 在原测试发送重新使能空请求之前，检查冻结目标距离和转角。
        if not self.setpoint_validated:
            try:
                (p, q), _ = self.get_actuals()
                self.validate_goal(p, q)
                self.setpoint_validated = True
            except (ValueError, TransformException) as exc:
                return str(exc)
        return None

    def validate_goal(self, position, quaternion):
        angle = (0.0 if self.position_only else
                 angular_error_deg(quaternion, self.goal_quaternion))
        distance = math.dist(position, self.args.xyz) * 1000
        if distance > self.args.max_displacement_mm:
            raise ValueError(f'冻结目标距离 {distance:.2f}mm / 转角 {angle:.2f}°，'
                             f'超过 --max-displacement-mm={self.args.max_displacement_mm}')
        if angle > self.max_angle:
            raise ValueError(f'目标转角 {angle:.2f}° 超过 --max-target-angle-deg={self.max_angle}')

    def make_target(self, actual, quaternion, right_actual):
        if self.position_only:
            self.goal_quaternion = quaternion
        self.validate_goal(actual, quaternion)
        super().make_target(actual, quaternion, right_actual)
        self.target_quaternion = self.goal_quaternion
        source = '保持当前姿态' if self.position_only else '使用标定姿态'
        self.get_logger().info(f'TARGET quaternion xyzw={self.target_quaternion}；{source}')
        if self.use_absolute_error_settle():
            self.get_logger().info(
                f'PASS 判定：位置误差≤{self.args.position_tolerance_mm:.3f}mm，'
                f'姿态误差≤{self.args.angle_tolerance_deg:.3f}°，'
                f'连续满足 {self.args.settle_time:.3f}s')
        else:
            self.get_logger().info(
                'PASS 判定：由上层脚本根据连续观察窗口内的 tool 位姿抖动决定')


def tag_parse_args():
    parser = argparse.ArgumentParser(description=__doc__, epilog='其余选项与 left_tool_accuracy_test.py 相同，例如 --execute --timeout 30 --max-displacement-mm 50')
    parser.add_argument('--tag-id', type=int, default=5)
    parser.add_argument('--tag-size', type=float, default=0.016)
    parser.add_argument('--image-topic', default='/wrist_left_camera/camera/color/image_raw')
    parser.add_argument('--calibration-file', default=str(WORKSPACE / 'calibration_data/luna_left_extrinsic02_home_20261009_verified.yaml'))
    parser.add_argument('--intrinsics-file', default=str(WORKSPACE / 'calibration_data/left_d405_intrinsics.yaml'))
    parser.add_argument('--offset-config', default=str(Path(__file__).resolve().with_name('tag5_left_tool_offset.yaml')))
    parser.add_argument('--capture-timeout', type=float, default=30.0)
    parser.add_argument('--max-target-angle-deg', type=float, default=90.0)
    parser.add_argument(
        '--position-tolerance-mm', type=float, default=1.0,
        help='到位位置阈值，单位 mm，默认恢复为 1.0；只影响 PASS 判定')
    parser.add_argument(
        '--angle-tolerance-deg', type=float, default=0.5,
        help='到位姿态阈值，单位 deg，默认恢复为 0.5；只影响 PASS 判定')
    parser.add_argument(
        '--tag-position-only', action='store_true',
        help='直接使用 Tag 中心位置，不使用 Tag→left_tool 偏移，并保持当前姿态')
    args, rest = parser.parse_known_args()
    for key in (
            'tag_size', 'capture_timeout', 'max_target_angle_deg',
            'position_tolerance_mm', 'angle_tolerance_deg'):
        if not math.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
            parser.error(f'{key} 必须为有限正数')
    if any(x.split('=')[0] in ('--xyz', '--offset', '--offset-frame') for x in rest):
        parser.error('目标由 AprilTag 和标定偏移生成，不能指定 --xyz/--offset')
    original = sys.argv
    try:
        sys.argv = [original[0]] + rest
        motion = accuracy_parse_args()
    finally:
        sys.argv = original
    motion.max_target_angle_deg = args.max_target_angle_deg
    motion.tag_position_only = args.tag_position_only
    motion.position_tolerance_mm = args.position_tolerance_mm
    motion.angle_tolerance_deg = args.angle_tolerance_deg
    return args, motion


# ==========================================================================
# 第 3 部分：抓取、返回快照位姿与底盘回位主流程（脚本入口）
# 来源：原 apriltag_left_tool_grasp_test.py
# ==========================================================================


GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 1.0
GRIPPER_START_HOLD_SEC = 0.5
GRIPPER_END_HOLD_SEC = 2.0
DEFAULT_RETURN_POSE_FILE = Path(__file__).resolve().with_name(
    'left_tool_joint_snapshot_20261010.yaml')
DEFAULT_BASE_RETURN_WORLD_Y_M = 0.22712
DEFAULT_BASE_RETURN_WORLD_X_M = 0.04681
DEFAULT_BASE_RETURN_WORLD_Z_M = 0.00603
DEFAULT_BASE_RETURN_WORLD_YAW_RAD = 0.05024


def option_present(arguments, name):
    return any(value == name or value.startswith(name + '=')
               for value in arguments)


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog=(
            'AprilTag 和运动参数与 apriltag_left_tool_accuracy_test.py 相同。'
            '夹爪脚本同时检查连续窗口内的 tool 抖动和目标误差；'
            '--position-tolerance-mm/--angle-tolerance-deg 用于到位判定。'))
    parser.add_argument(
        '--gripper-duration', type=float, default=8.0,
        help='夹爪命令从 0.0 线性增加到 1.0 的时间，默认 8 s')
    parser.add_argument(
        '--gripper-rate-hz', type=float, default=10.0,
        help='夹爪命令发布频率，默认 10 Hz，允许范围 1..20 Hz')
    parser.add_argument(
        '--gripper-topic', default='/tj/control/gripperValueL',
        help='左夹爪命令话题')
    parser.add_argument(
        '--gripper-feedback-topic', default='/info/gripper_feedback_L',
        help='左夹爪反馈话题')
    parser.add_argument(
        '--gripper-status-topic', default='/info/gripper_feedback_L_err',
        help='左夹爪状态码话题')
    parser.add_argument(
        '--stability-window-sec', type=float, default=1.5,
        help='连续位姿稳定判定窗口，单位 s，默认 1.5 s')
    parser.add_argument(
        '--stability-position-jitter-mm', type=float, default=0.5,
        help='窗口内 tool 位置最大抖动，单位 mm，默认 0.5 mm')
    parser.add_argument(
        '--stability-angle-jitter-deg', type=float, default=0.3,
        help='窗口内 tool 姿态最大抖动，单位 deg，默认 0.3 deg')
    parser.add_argument(
        '--return-speed-mm-s', type=float, default=20.0,
        help='夹爪闭合后返回 YAML 记录位姿的线速度，默认 20 mm/s')
    parser.add_argument(
        '--approach-height-mm', type=float, default=30.0,
        help='抓取前先到目标上方的高度，单位 mm，默认 30 mm；设为 0 关闭')
    parser.add_argument(
        '--grasp-and-lift', action='store_true',
        help='抓取并抬升 50 mm 后结束；跳过返回、底盘移动、松爪和 go_home')
    parser.add_argument(
        '--return-pose-file', default=str(DEFAULT_RETURN_POSE_FILE),
        help='返回位姿快照 YAML，默认 left_tool_joint_snapshot_20261010.yaml')
    parser.add_argument(
        '--base-right-distance-mm', type=float, default=350.0,
        help='手臂返回后底盘向右横移的名义距离，默认 350 mm')
    parser.add_argument(
        '--base-lateral-speed-m-s', type=float, default=0.06,
        help='底盘横移速度绝对值，默认 0.06 m/s，范围 0.06..0.10 m/s')
    parser.add_argument(
        '--base-return-world-x-m', type=float,
        default=DEFAULT_BASE_RETURN_WORLD_X_M,
        help='最后反向移动的底盘世界坐标 x，默认 0.04681 m')
    parser.add_argument(
        '--base-return-world-y-m', type=float,
        default=DEFAULT_BASE_RETURN_WORLD_Y_M,
        help='最后反向移动的底盘世界坐标 y，默认 0.22712 m')
    parser.add_argument(
        '--base-return-world-z-m', type=float,
        default=DEFAULT_BASE_RETURN_WORLD_Z_M,
        help='最后反向移动的底盘世界坐标 z，默认 0.00603 m；仅检查，底盘接口不主动调 z')
    parser.add_argument(
        '--base-return-world-yaw-rad', type=float,
        default=DEFAULT_BASE_RETURN_WORLD_YAW_RAD,
        help='最后反向移动的底盘世界 yaw，默认 0.05024 rad（2.88 deg）')
    parser.add_argument(
        '--base-return-yaw-speed-rad-s', type=float, default=0.10,
        help='最后反向阶段底盘 yaw 闭环速度，默认 0.10 rad/s')
    parser.add_argument(
        '--base-return-pose-tolerance-m', type=float, default=0.03,
        help='底盘世界坐标位置误差容限，默认 0.03 m')
    parser.add_argument(
        '--base-return-yaw-tolerance-rad', type=float, default=0.04,
        help='底盘世界 yaw 误差容限，默认 0.04 rad')
    parser.add_argument(
        '--vlahost-url', default='http://127.0.0.1:8011',
        help='VLAHost HTTP 地址，默认 http://127.0.0.1:8011')

    gripper, remaining = parser.parse_known_args()
    defaults = (
        ('--position-tolerance-mm', '10.0'),
        ('--angle-tolerance-deg', '2.5'),
        ('--settle-time', '1.0'),
    )
    for name, value in defaults:
        if not option_present(remaining, name):
            remaining.extend((name, value))

    original = sys.argv
    try:
        sys.argv = [original[0]] + remaining
        tag_args, motion = tag_parse_args()
    finally:
        sys.argv = original

    if not math.isfinite(gripper.gripper_duration) or gripper.gripper_duration <= 0.0:
        parser.error('--gripper-duration 必须为有限正数')
    if (not math.isfinite(gripper.gripper_rate_hz) or
            not 1.0 <= gripper.gripper_rate_hz <= 20.0):
        parser.error('--gripper-rate-hz 必须在 1..20 Hz 内')
    for name in (
            'stability_window_sec', 'stability_position_jitter_mm',
            'stability_angle_jitter_deg', 'return_speed_mm_s',
            'base_right_distance_mm', 'base_lateral_speed_m_s',
            'base_return_world_x_m', 'base_return_world_y_m',
            'base_return_world_z_m',
            'base_return_world_yaw_rad', 'base_return_yaw_speed_rad_s',
            'base_return_pose_tolerance_m', 'base_return_yaw_tolerance_rad'):
        value = getattr(gripper, name)
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f'--{name.replace("_", "-")} 必须为有限正数')
    if not 0.06 <= gripper.base_lateral_speed_m_s <= 0.10:
        parser.error('--base-lateral-speed-m-s 必须在 0.06..0.10 m/s 内')
    if gripper.base_return_yaw_speed_rad_s > 0.20:
        parser.error('--base-return-yaw-speed-rad-s 不能超过底盘 0.20 rad/s 限制')
    if (not math.isfinite(gripper.approach_height_mm) or
            gripper.approach_height_mm < 0.0):
        parser.error('--approach-height-mm 必须为大于等于 0 的有限数值')
    return tag_args, motion, gripper


def load_return_pose(path):
    """读取并校验快照中 base_link -> left_tool 的目标位姿。"""
    snapshot_path = Path(path).expanduser()
    try:
        with snapshot_path.open(encoding='utf-8') as stream:
            snapshot = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f'无法读取返回位姿文件 {snapshot_path}：{error}') from error

    try:
        pose = snapshot['tool_pose']
        position = tuple(float(value) for value in pose['xyz_m'])
        quaternion = tuple(float(value) for value in pose['quaternion_xyzw'])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f'返回位姿文件字段无效：{error}') from error
    if pose.get('base_frame') != 'base_link' or pose.get('tool_frame') != 'left_tool':
        raise ValueError('返回位姿文件必须记录 base_link -> left_tool')
    if (len(position) != 3 or len(quaternion) != 4 or
            not all(math.isfinite(value) for value in position + quaternion)):
        raise ValueError('返回位姿必须包含有限的 xyz_m[3] 和 quaternion_xyzw[4]')
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-12 or abs(norm - 1.0) > 1e-3:
        raise ValueError(f'返回位姿四元数模长无效：{norm:.6f}')
    quaternion = tuple(value / norm for value in quaternion)
    return snapshot_path, position, quaternion


def request_json(base_url, path, body, timeout=3.0):
    """调用本机 VLAHost，并将 HTTP/JSON 错误转换为可读异常。"""
    data = json.dumps(body).encode('utf-8')
    request = Request(
        base_url.rstrip('/') + path, data=data,
        headers={'Content-Type': 'application/json'})
    try:
        with urlopen(request, timeout=timeout) as response:
            return json.load(response)
    except HTTPError as error:
        detail = error.read().decode('utf-8', errors='replace')
        raise ValueError(f'VLAHost HTTP {error.code}: {detail}') from error
    except (URLError, OSError, json.JSONDecodeError) as error:
        raise ValueError(f'无法访问 VLAHost：{error}') from error


def get_json(base_url, path, timeout=3.0):
    """读取 VLAHost 状态接口。"""
    request = Request(base_url.rstrip('/') + path, method='GET')
    try:
        with urlopen(request, timeout=timeout) as response:
            return json.load(response)
    except (HTTPError, URLError, OSError, json.JSONDecodeError) as error:
        raise ValueError(f'无法读取 VLAHost 状态：{error}') from error


class TagGrasp(TagAccuracy):
    """执行抓取、返回快照位姿，以及完成后的底盘右移。"""

    def __init__(self, motion, goal, gripper, return_position,
                 return_quaternion, return_pose_file):
        self.gripper_args = gripper
        self.phase = 'approach'
        self.close_started_at = None
        self.return_started_at = None
        self.return_start_position = None
        self.return_goal_position = None
        self.return_start_quaternion = None
        self.return_goal_quaternion = None
        self.return_duration = 0.0
        self.base_move_started_at = None
        self.base_move_duration = 0.0
        self.base_move_sequence = None
        self.last_base_move_poll_at = float('-inf')
        self.base_yaw_started_at = None
        self.base_yaw_direction_sign = 1.0
        self.base_yaw_direction_locked = False
        self.base_yaw_last_yaw = None
        self.base_yaw_last_command = 0.0
        self.base_yaw_last_motion_at = None
        self.base_return_world_pose = None
        self.open_started_at = None
        self.recorded_return_position = tuple(return_position)
        self.recorded_return_quaternion = tuple(return_quaternion)
        self.return_pose_file = str(return_pose_file)
        self.last_gripper_publish_at = float('-inf')
        self.last_gripper_command = None
        self.last_grasp_log_at = float('-inf')
        self.gripper_feedback = None
        self.gripper_feedback_at = None
        self.gripper_status = None
        self.gripper_status_at = None
        self.stability_samples = deque()
        self.last_stability_log_at = float('-inf')
        self.go_home_client = None
        self.go_home_future = None
        self.go_home_started_at = None
        self.lift_started_at = None
        self.lift_start_position = None
        self.lift_goal_position = None
        self.lift_start_quaternion = None
        self.lift_duration = 0.0
        self.grasp_and_lift = bool(gripper.grasp_and_lift)
        self.approach_waypoint_active = False
        self.approach_final_position = None
        super().__init__(motion, goal)

        # 直接调用 Apex QP 控制器提供的 ROS 服务，不经过 VLAHost HTTP 接口。
        self.go_home_client = self.create_client(
            Trigger,
            topic_name(motion.namespace, 'control/go_home'))

        self.gripper_publisher = self.create_publisher(
            Float32, gripper.gripper_topic, 10)
        self.create_subscription(
            Float32MultiArray, gripper.gripper_feedback_topic,
            self.on_gripper_feedback, 20)
        self.create_subscription(
            Int32MultiArray, gripper.gripper_status_topic,
            self.on_gripper_status, 20)
        self.get_logger().info(
            '夹爪动作：手臂稳定后 0.0 → 1.0，闭合时间 %.1f s，频率 %.1f Hz' %
            (gripper.gripper_duration, gripper.gripper_rate_hz))
        self.get_logger().info(
            '路径：直接移动到 AprilTag 转换得到的最终 left_tool 目标，'
            '先到目标上方，再下降到抓取点；保持姿态到位后再闭合夹爪')
        self.get_logger().info(
            '稳定平台判定：连续 %.1fs 窗口内 tool 抖动≤%.2fmm/%.2f°；'
            '同时要求到目标误差≤%.2fmm/%.2f°' %
            (gripper.stability_window_sec,
             gripper.stability_position_jitter_mm,
             gripper.stability_angle_jitter_deg,
             motion.position_tolerance_mm, motion.angle_tolerance_deg))
        self.get_logger().info(
            '返回目标来自 %s：base_link -> left_tool xyz=[%.6f, %.6f, %.6f]' %
            (self.return_pose_file, *self.recorded_return_position))
        self.get_logger().info(
            '松爪后直接调用 ROS %s 执行真实机器人 home 服务' %
            topic_name(motion.namespace, 'control/go_home'))
        self.get_logger().info(
            '返回完成后底盘向右横移 %.1f mm：vy=-%.3f m/s，VLAHost=%s' %
            (gripper.base_right_distance_mm,
             gripper.base_lateral_speed_m_s, gripper.vlahost_url))
        self.get_logger().info(
            '最后反向返回底盘世界坐标：x=%.5f m，y=%.5f m，z=%.5f m（仅检查），'
            'yaw=%.5f rad（%.2f°）；'
            '通过 /base_world_pose 闭环' %
            (gripper.base_return_world_x_m,
             gripper.base_return_world_y_m,
             gripper.base_return_world_z_m,
             gripper.base_return_world_yaw_rad,
             math.degrees(gripper.base_return_world_yaw_rad)))

    def on_gripper_feedback(self, message):
        if len(message.data) < 3:
            return
        values = tuple(float(value) for value in message.data[:3])
        if all(math.isfinite(value) for value in values):
            self.gripper_feedback = values
            self.gripper_feedback_at = time.monotonic()

    def on_gripper_status(self, message):
        if message.data:
            self.gripper_status = tuple(int(value) for value in message.data)
            self.gripper_status_at = time.monotonic()

    def use_absolute_error_settle(self):
        """由窗口判定统一检查抖动和到位误差，避免父类提前切换阶段。"""
        return False

    def make_target(self, actual, quaternion, right_actual):
        """先到最终抓取点上方，再沿 base_link z 轴下降到目标点。"""
        super().make_target(actual, quaternion, right_actual)
        height_m = self.gripper_args.approach_height_mm / 1000.0
        if height_m <= 0.0:
            return
        self.approach_final_position = tuple(self.target)
        self.target = (
            self.approach_final_position[0],
            self.approach_final_position[1],
            self.approach_final_position[2] + height_m)
        self.approach_waypoint_active = True
        self.get_logger().warning(
            '抓取路径：先到 Tag 目标上方 %.1f mm，再下降到最终目标点；姿态保持标定目标' %
            self.gripper_args.approach_height_mm)

    @staticmethod
    def interpolate_quaternion(first, second, progress):
        """用最短弧线性插值四元数，避免返回时姿态突然跳变。"""
        dot = sum(a * b for a, b in zip(first, second))
        if dot < 0.0:
            second = tuple(-value for value in second)
            dot = -dot
        values = tuple(
            a + progress * (b - a) for a, b in zip(first, second))
        norm = math.sqrt(sum(value * value for value in values))
        if norm < 1e-12:
            return tuple(second)
        return tuple(value / norm for value in values)

    def other_gripper_publishers(self):
        result = []
        for endpoint in self.get_publishers_info_by_topic(
                self.gripper_args.gripper_topic):
            if (endpoint.node_name == self.get_name() and
                    endpoint.node_namespace == self.get_namespace()):
                continue
            full_name = '/'.join(
                part.strip('/') for part in
                (endpoint.node_namespace, endpoint.node_name) if part.strip('/'))
            result.append('/' + full_name)
        return sorted(set(result))

    def check_gripper(self, now):
        if self.gripper_publisher.get_subscription_count() == 0:
            return f'{self.gripper_args.gripper_topic} 没有订阅者'
        publishers = self.other_gripper_publishers()
        # if publishers:
        #     return ('夹爪命令话题存在其他发布者 %s；请先停止这些发布节点' %
        #             ', '.join(publishers))
        timeout = self.args.feedback_timeout
        if self.gripper_feedback_at is None or now - self.gripper_feedback_at > timeout:
            return '没有收到新鲜的左夹爪反馈'
        if self.gripper_status_at is None or now - self.gripper_status_at > timeout:
            return '没有收到新鲜的左夹爪状态码'
        # 达妙驱动中 1 表示 On/正常，0 表示 Off，8..14 表示各类故障。
        if any(value != 1 for value in self.gripper_status):
            return f'左夹爪状态码不是正常值 1：{list(self.gripper_status)}'
        return None

    def check_mux(self):
        problem = super().check_mux()
        if problem:
            return problem
        if self.args.execute:
            return self.check_gripper(time.monotonic())
        return None

    def finish(self, code, reason):
        if code == 0 and reason.startswith('PASS:'):
            if self.phase == 'approach':
                if self.approach_waypoint_active:
                    self.target = self.approach_final_position
                    self.approach_waypoint_active = False
                    self.reset_stability_window()
                    self.get_logger().warning(
                        '已到达 Tag 上方 %.1f mm，开始垂直下降到最终 Tag 目标点' %
                        self.gripper_args.approach_height_mm)
                    return
                self.stability_samples.clear()
                self.phase = 'closing'
                self.close_started_at = time.monotonic()
                self.settled_since = None
                self.get_logger().warning(
                    f'最终抓取点已稳定：{reason}；开始将左夹爪从 0.0 缓慢关闭到 1.0')
                return
            if self.phase == 'returning':
                self.start_base_right_move(reason)
                return
        if code != 0 and self.phase in (
                'base_moving', 'base_returning', 'base_yaw_returning'):
            self.stop_base_motion()
        super().finish(code, reason)

    def start_base_right_move(self, arm_reason):
        """手臂返回完成后，通过 VLAHost 启动闭环世界坐标右移。"""
        distance_m = self.gripper_args.base_right_distance_mm / 1000.0
        body = {
            'offset_xy_m': [0.0, -distance_m],
            'speed_mps': self.gripper_args.base_lateral_speed_m_s,
        }
        try:
            result = request_json(
                self.gripper_args.vlahost_url, '/base_move_world', body)
        except ValueError as error:
            super().finish(3, f'手臂已返回，但底盘右移启动失败：{error}')
            return
        move = result.get('move', {})
        if not result.get('success') or move.get('status') != 'active':
            super().finish(
                3, '手臂已返回，但 VLAHost 未主动执行底盘动作：'
                f'{result}')
            return
        self.phase = 'base_moving'
        self.base_move_started_at = time.monotonic()
        self.base_move_duration = self.gripper_args.base_right_distance_mm / self.gripper_args.base_lateral_speed_m_s
        self.base_move_sequence = move
        self.get_logger().warning(
            '%s；底盘闭环向右横移 %.1f mm，offset_xy_m=[0.0, %.3f]，'
            'speed=%.3f m/s，VLAHost=%s' %
            (arm_reason, self.gripper_args.base_right_distance_mm,
             -distance_m, self.gripper_args.base_lateral_speed_m_s,
             self.gripper_args.vlahost_url))

    def start_base_return_move(self, arm_reason):
        """底盘反向阶段闭环回到指定世界 x/y；完成后再闭环修正 yaw。"""
        try:
            pose = get_json(self.gripper_args.vlahost_url, '/base_world_pose').get('pose')
            self.validate_base_world_pose(pose)
        except ValueError as error:
            super().finish(3, f'底盘反向返回前无法读取世界坐标：{error}')
            return

        current_x = float(pose['position']['x'])
        current_y = float(pose['position']['y'])
        offset_x = self.gripper_args.base_return_world_x_m - current_x
        offset_y = self.gripper_args.base_return_world_y_m - current_y
        self.base_return_world_pose = pose
        offset_distance = math.hypot(offset_x, offset_y)
        # /base_move_world 要求单次位移至少 50 mm；小误差直接跳过平移，随后修正 yaw。
        if offset_distance <= 0.03:
            self.get_logger().info(
                '底盘 x/y 已在目标容限内：current=[%.5f, %.5f] m '
                'target=[%.5f, %.5f] m；跳过平移' %
                (current_x, current_y,
                 self.gripper_args.base_return_world_x_m,
                 self.gripper_args.base_return_world_y_m))
            self.start_base_yaw_return(arm_reason)
            return
        if offset_distance < 0.05:
            super().finish(
                3, '底盘世界 x/y 误差 %.3f m 位于 0.03..0.05 m，'
                '/base_move_world 最小单次位移为 0.05 m，未强行越过目标' %
                offset_distance)
            return
        if offset_distance > 0.50:
            super().finish(
                3, '底盘世界 x/y 需要移动 %.3f m，超过 /base_move_world 单次 0.50 m 限制' %
                offset_distance)
            return
        body = {
            'offset_xy_m': [offset_x, offset_y],
            'speed_mps': self.gripper_args.base_lateral_speed_m_s,
        }
        try:
            result = request_json(
                self.gripper_args.vlahost_url, '/base_move_world', body)
        except ValueError as error:
            super().finish(3, f'手臂回 home 后底盘世界 x/y 返回失败：{error}')
            return
        move = result.get('move', {})
        if not result.get('success') or move.get('status') != 'active':
            super().finish(3, f'底盘世界 x/y 返回未启动：{result}')
            return
        self.phase = 'base_returning'
        self.base_move_started_at = time.monotonic()
        self.last_base_move_poll_at = float('-inf')
        self.get_logger().warning(
            '%s；底盘闭环返回世界 x/y：current=[%.5f, %.5f] m，'
            'target=[%.5f, %.5f] m，offset=[%.3f, %.3f] m，speed=%.3f m/s' %
            (arm_reason, current_x, current_y,
             self.gripper_args.base_return_world_x_m,
             self.gripper_args.base_return_world_y_m,
             offset_x, offset_y, self.gripper_args.base_lateral_speed_m_s))

    @staticmethod
    def validate_base_world_pose(pose):
        if not isinstance(pose, dict) or not pose.get('valid'):
            raise ValueError('世界底盘位姿无效')
        if pose.get('frame_id') != 'camera_init' or pose.get('child_frame_id') != 'body':
            raise ValueError(
                '世界底盘位姿坐标系异常，需要 camera_init -> body，实际为 '
                f'{pose.get("frame_id")} -> {pose.get("child_frame_id")}')
        if pose.get('received_age_s') is not None and pose['received_age_s'] > 0.5:
            raise ValueError(f'世界底盘位姿过期：{pose["received_age_s"]:.3f} s')
        position = pose.get('position')
        if not isinstance(position, dict) or not all(
                math.isfinite(float(position.get(axis))) for axis in ('x', 'y', 'z')):
            raise ValueError('世界底盘位置包含非有限值')
        if not math.isfinite(float(pose.get('yaw_rad'))):
            raise ValueError('世界底盘 yaw 无效')

    @staticmethod
    def wrapped_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def start_base_yaw_return(self, arm_reason):
        """平移完成后，使用 VLAHost base_velocity 闭环修正世界 yaw。"""
        try:
            pose = get_json(self.gripper_args.vlahost_url, '/base_world_pose').get('pose')
            self.validate_base_world_pose(pose)
        except ValueError as error:
            super().finish(3, f'底盘 yaw 返回前无法读取世界坐标：{error}')
            return
        yaw = float(pose['yaw_rad'])
        target = self.gripper_args.base_return_world_yaw_rad
        error = self.wrapped_angle(target - yaw)
        self.base_return_world_pose = pose
        if abs(error) <= self.gripper_args.base_return_yaw_tolerance_rad:
            self.finish_base_return(arm_reason, pose)
            return
        self.phase = 'base_yaw_returning'
        self.base_yaw_started_at = time.monotonic()
        self.base_yaw_direction_sign = 1.0
        self.base_yaw_direction_locked = False
        self.base_yaw_last_yaw = yaw
        self.base_yaw_last_command = 0.0
        self.base_yaw_last_motion_at = self.base_yaw_started_at
        self.last_base_move_poll_at = float('-inf')
        self.get_logger().warning(
            '%s；底盘开始闭环修正世界 yaw：current=%.5f rad target=%.5f rad error=%.5f rad' %
            (arm_reason, yaw, target, error))

    def finish_base_return(self, arm_reason, pose):
        self.validate_base_world_pose(pose)
        x = float(pose['position']['x'])
        y = float(pose['position']['y'])
        z = float(pose['position']['z'])
        yaw = float(pose['yaw_rad'])
        x_error = x - self.gripper_args.base_return_world_x_m
        y_error = y - self.gripper_args.base_return_world_y_m
        z_error = z - self.gripper_args.base_return_world_z_m
        yaw_error = self.wrapped_angle(
            yaw - self.gripper_args.base_return_world_yaw_rad)
        self.get_logger().info(
            '底盘世界目标完成：actual x=%.5f y=%.5f z=%.5f yaw=%.5f rad；'
            'error x=%.4f y=%.4f z=%.4f yaw=%.5f rad' %
            (x, y, z, yaw, x_error, y_error, z_error, yaw_error))
        if math.hypot(x_error, y_error) > self.gripper_args.base_return_pose_tolerance_m:
            super().finish(
                3, '底盘返回后 x/y 误差过大：%.4f m（x=%.4f, y=%.4f）' %
                (math.hypot(x_error, y_error), x_error, y_error))
            return
        if abs(z_error) > self.gripper_args.base_return_pose_tolerance_m:
            super().finish(
                3, '底盘返回后 z 与目标不一致（底盘接口不能主动调 z）：'
                f'actual={z:.5f} target={self.gripper_args.base_return_world_z_m:.5f}')
            return
        if abs(yaw_error) > self.gripper_args.base_return_yaw_tolerance_rad:
            super().finish(3, f'底盘返回后 yaw 误差过大：{yaw_error:.5f} rad')
            return
        self.start_go_home()

    def stop_base_motion(self):
        """用零速度块覆盖尚未结束的底盘动作。"""
        if self.phase == 'base_yaw_returning':
            try:
                self.publish_base_yaw_velocity(0.0)
            except ValueError as error:
                self.get_logger().error(f'发送底盘 yaw 零速度失败：{error}')
        if self.base_move_started_at is None:
            return
        try:
            request_json(
                self.gripper_args.vlahost_url, '/base_move_world/cancel', {})
        except ValueError as error:
            self.get_logger().error(f'发送底盘零速度失败：{error}')
        self.base_move_started_at = None

    def publish_base_yaw_velocity(self, velocity):
        """通过 VLAHost 单步发送底盘 yaw 速度，发送失败由调用方处理。"""
        return request_json(
            self.gripper_args.vlahost_url, '/action',
            {'base_velocity': [0.0, 0.0, float(velocity)]})

    def base_yaw_return_tick(self):
        """闭环修正底盘世界 yaw，达到目标后停止并校验 x/y/z/yaw。"""
        now = time.monotonic()
        try:
            pose = get_json(self.gripper_args.vlahost_url, '/base_world_pose').get('pose')
            self.validate_base_world_pose(pose)
        except ValueError as error:
            super().finish(3, f'底盘 yaw 返回期间读取世界坐标失败：{error}')
            return
        yaw = float(pose['yaw_rad'])
        yaw_error = self.wrapped_angle(
            self.gripper_args.base_return_world_yaw_rad - yaw)
        if self.base_yaw_last_yaw is not None:
            yaw_delta = self.wrapped_angle(yaw - self.base_yaw_last_yaw)
            if (not self.base_yaw_direction_locked and
                    abs(yaw_delta) >= 0.003 and
                    abs(self.base_yaw_last_command) > 1e-9):
                if yaw_delta * self.base_yaw_last_command < 0.0:
                    self.base_yaw_direction_sign = -1.0
                self.base_yaw_direction_locked = True
                self.get_logger().warning(
                    '已根据第一次有效 yaw 反馈锁定底盘命令方向：sign=%+.0f，'
                    '后续不再重复切换' % self.base_yaw_direction_sign)
                self.base_yaw_last_motion_at = now
            self.base_yaw_last_yaw = yaw
        if abs(yaw_error) <= self.gripper_args.base_return_yaw_tolerance_rad:
            try:
                self.publish_base_yaw_velocity(0.0)
            except ValueError:
                pass
            self.finish_base_return('底盘 x/y 返回完成', pose)
            return
        if now - self.base_yaw_started_at > 10.0:
            try:
                self.publish_base_yaw_velocity(0.0)
            except ValueError:
                pass
            super().finish(3, f'底盘 yaw 返回超时，剩余误差 {yaw_error:.5f} rad')
            return
        if now - self.base_yaw_last_motion_at > 3.0:
            try:
                self.publish_base_yaw_velocity(0.0)
            except ValueError:
                pass
            super().finish(
                3, '底盘 yaw 命令发送后 3 s 内世界 yaw 没有有效变化；'
                f'剩余误差 {yaw_error:.5f} rad')
            return
        if now - self.last_base_move_poll_at < 0.1:
            return
        self.last_base_move_poll_at = now
        speed = math.copysign(
            min(self.gripper_args.base_return_yaw_speed_rad_s,
                max(0.06, abs(yaw_error) / 0.1)), yaw_error)
        speed *= self.base_yaw_direction_sign
        try:
            result = self.publish_base_yaw_velocity(speed)
        except ValueError as error:
            super().finish(3, f'底盘 yaw 速度命令失败：{error}')
            return
        if not result.get('success'):
            super().finish(3, f'底盘 yaw 速度命令未执行：{result}')
            return
        self.base_yaw_last_command = speed

    def start_go_home(self):
        """底盘反向返回完成后，直接调用 ROS go_home 服务。"""
        self.phase = 'go_home'
        service_name = self.go_home_client.srv_name
        self.get_logger().warning(
            f'夹爪已松开；直接调用 ROS {service_name} 返回左臂 home')
        if not self.go_home_client.wait_for_service(timeout_sec=2.0):
            super().finish(3, f'ROS go home 服务不可用：{service_name}')
            return
        self.go_home_started_at = time.monotonic()
        self.go_home_future = self.go_home_client.call_async(Trigger.Request())

    def start_lift_after_open(self):
        """松爪后先沿 base_link 的 z 轴向上移动 50 mm。"""
        try:
            (position, quaternion), _ = self.get_actuals()
        except (TransformException, ValueError) as error:
            super().finish(3, f'松爪后无法读取 left_tool 位姿，不能上移：{error}')
            return
        self.lift_start_position = tuple(position)
        self.lift_goal_position = (
            position[0], position[1], position[2] + 0.050)
        self.lift_start_quaternion = tuple(quaternion)
        self.lift_duration = 50.0 / self.gripper_args.return_speed_mm_s
        self.lift_started_at = time.monotonic()
        self.phase = 'lifting'
        self.target = self.lift_start_position
        self.target_quaternion = self.lift_start_quaternion
        self.get_logger().warning(
            '夹爪已松开；先以 %.1f mm/s 向上移动 50 mm，预计 %.1f s，'
            '之后底盘反向返回，最后执行 ROS go_home' %
            (self.gripper_args.return_speed_mm_s, self.lift_duration))

    def start_lift_after_grasp(self, position, quaternion):
        """抓取保持后闭合夹爪，沿 base_link z 轴向上移动 50 mm 并结束。"""
        self.lift_start_position = tuple(position)
        self.lift_goal_position = (
            position[0], position[1], position[2] + 0.050)
        self.lift_start_quaternion = tuple(quaternion)
        self.lift_duration = 50.0 / self.gripper_args.return_speed_mm_s
        self.lift_started_at = time.monotonic()
        self.phase = 'lifting_after_grasp'
        self.target = self.lift_start_position
        self.target_quaternion = self.lift_start_quaternion
        self.get_logger().warning(
            'Tag 已抓取并保持夹爪闭合；以 %.1f mm/s 向上移动 50 mm，完成后结束任务' %
            self.gripper_args.return_speed_mm_s)

    def lift_tick(self):
        """执行松爪后的 50 mm 上移动作。"""
        now = time.monotonic()
        problem = self.check_mux()
        if problem:
            super().finish(3, '松爪后上移期间安全检查失败：' + problem)
            return
        self.publish_gripper(GRIPPER_OPEN, now)
        progress = min(
            1.0, max(0.0, (now - self.lift_started_at) / self.lift_duration))
        self.target = tuple(
            start + progress * (goal - start)
            for start, goal in zip(self.lift_start_position, self.lift_goal_position))
        self.target_quaternion = self.lift_start_quaternion
        self.publish_target()
        if progress >= 1.0:
            self.start_base_return_move('松爪后已向上移动 50 mm')

    def lift_after_grasp_tick(self):
        """保持夹爪闭合，执行抓取后的 50 mm 上移并结束。"""
        now = time.monotonic()
        problem = self.check_mux()
        if problem:
            super().finish(3, '抓取后上移期间安全检查失败：' + problem)
            return
        self.publish_gripper(GRIPPER_CLOSED, now)
        progress = min(
            1.0, max(0.0, (now - self.lift_started_at) / self.lift_duration))
        self.target = tuple(
            start + progress * (goal - start)
            for start, goal in zip(self.lift_start_position, self.lift_goal_position))
        self.target_quaternion = self.lift_start_quaternion
        self.publish_target()
        if progress >= 1.0:
            super().finish(0, 'PASS: Tag 已抓取并抬升 50 mm；保持夹爪闭合')

    def go_home_tick(self):
        """等待 ROS Trigger 返回，避免阻塞夹爪/底盘状态机。"""
        if self.go_home_future is None:
            return
        if (time.monotonic() - self.go_home_started_at) > 35.0:
            self.go_home_future = None
            super().finish(3, 'ROS go home 服务调用超时（35 s）')
            return
        if not self.go_home_future.done():
            return
        future = self.go_home_future
        self.go_home_future = None
        try:
            response = future.result()
        except Exception as error:
            super().finish(3, f'ROS go home 服务异常：{error}')
            return
        if not response.success:
            super().finish(3, f'ROS go home 返回失败：{response.message}')
            return
        super().finish(
            0, f'PASS: 底盘已返回原位置；ROS go_home 已启动：'
            f'{response.message or "success"}')

    def publish_gripper(self, value, now):
        period = 1.0 / self.gripper_args.gripper_rate_hz
        if now - self.last_gripper_publish_at + 1e-9 < period:
            return
        value = min(GRIPPER_CLOSED, max(GRIPPER_OPEN, float(value)))
        self.gripper_publisher.publish(Float32(data=value))
        self.last_gripper_command = value
        self.last_gripper_publish_at = now

    def fail_closing(self, reason):
        suffix = '' if self.last_gripper_command is None else (
            f'；停止增加闭合量，最后命令为 {self.last_gripper_command:.3f}')
        super().finish(3, reason + suffix)

    def closing_tick(self):
        now = time.monotonic()
        try:
            (actual, quaternion), _ = self.get_actuals()
        except (TransformException, ValueError) as error:
            if (self.last_valid_tf is None or
                    now - self.last_valid_tf > self.args.feedback_timeout):
                self.fail_closing(f'闭合期间 TF 失效：{error}')
            return
        self.last_valid_tf = now

        problem = self.check_mux()
        if problem:
            self.fail_closing('闭合期间安全检查失败：' + problem)
            return
        self.publish_target()

        position_mm = math.dist(actual, self.target) * 1000.0
        angle_deg = angular_error_deg(
            quaternion, self.target_quaternion)
        # 到位阈值外再留少量闭合扰动余量，超出即停止继续夹紧。
        position_limit = max(40.0, self.args.position_tolerance_mm)
        angle_limit = max(8.0, self.args.angle_tolerance_deg)
        if position_mm > position_limit or angle_deg > angle_limit:
            self.fail_closing(
                '闭合期间手臂偏离目标：%.3f mm / %.3f deg，限制 %.3f mm / %.3f deg' %
                (position_mm, angle_deg, position_limit, angle_limit))
            return

        elapsed = now - self.close_started_at
        if elapsed < GRIPPER_START_HOLD_SEC:
            command = GRIPPER_OPEN
            state = '起点保持'
        else:
            ramp_elapsed = elapsed - GRIPPER_START_HOLD_SEC
            progress = min(1.0, ramp_elapsed / self.gripper_args.gripper_duration)
            command = GRIPPER_OPEN + (GRIPPER_CLOSED - GRIPPER_OPEN) * progress
            state = '缓慢闭合' if progress < 1.0 else '闭合保持'

        self.publish_gripper(command, now)
        if now - self.last_grasp_log_at >= 1.0:
            self.last_grasp_log_at = now
            position, velocity, torque = self.gripper_feedback
            self.get_logger().info(
                'GRASP %s command=%.3f feedback=[pos=%.5f vel=%.5f torque=%.5f] '
                'arm_error=%.3f mm / %.3f deg' %
                (state, command, position, velocity, torque,
                 position_mm, angle_deg))

        finish_time = (GRIPPER_START_HOLD_SEC +
                       self.gripper_args.gripper_duration +
                       GRIPPER_END_HOLD_SEC)
        if elapsed >= finish_time:
            self.publish_gripper(GRIPPER_CLOSED, now)
            if self.grasp_and_lift:
                self.start_lift_after_grasp(actual, quaternion)
                return
            self.phase = 'returning'
            self.return_started_at = now
            self.motion_start = now
            self.return_start_position = tuple(actual)
            self.return_goal_position = self.recorded_return_position
            self.return_start_quaternion = tuple(quaternion)
            self.return_goal_quaternion = self.recorded_return_quaternion
            return_distance_mm = math.dist(
                self.return_start_position, self.return_goal_position) * 1000.0
            if return_distance_mm > self.args.max_displacement_mm:
                self.fail_closing(
                    'YAML 坐标距抓取位置 %.3f mm，超过 '
                    '--max-displacement-mm=%.3f' %
                    (return_distance_mm, self.args.max_displacement_mm))
                return
            self.return_duration = (
                return_distance_mm / self.gripper_args.return_speed_mm_s)
            self.target = self.return_start_position
            self.target_quaternion = self.interpolate_quaternion(
                quaternion, self.return_goal_quaternion, 0.0)
            self.reset_stability_window()
            self.get_logger().warning(
                '夹爪已完全闭合并保持 %.1f s；以 %.1f mm/s 慢速移动到 YAML 坐标，'
                '预计 %.1f s' %
                (GRIPPER_END_HOLD_SEC, self.gripper_args.return_speed_mm_s,
                 self.return_duration))

    def opening_tick(self):
        """底盘右移完成后，将夹爪从 1.0 缓慢打开到 0.0。"""
        now = time.monotonic()
        problem = self.check_mux()
        if problem:
            super().finish(3, '松爪期间安全检查失败：' + problem)
            return
        self.publish_target()
        elapsed = now - self.open_started_at
        progress = min(1.0, max(0.0, elapsed / self.gripper_args.gripper_duration))
        self.publish_gripper(GRIPPER_CLOSED * (1.0 - progress), now)
        if progress >= 1.0:
            self.publish_gripper(GRIPPER_OPEN, now)
            self.start_lift_after_open()

    def reset_stability_window(self):
        self.stability_samples.clear()

    def maybe_accept_stable_plateau(self):
        """末端同时稳定且到位才切换阶段，避免把停滞误判为到达。"""
        if (self.exit_code is not None or self.target is None or
                self.phase not in ('approach', 'returning')):
            self.reset_stability_window()
            return

        try:
            (position, quaternion), _ = self.get_actuals()
        except (TransformException, ValueError):
            self.reset_stability_window()
            return

        now = time.monotonic()
        self.stability_samples.append(
            (now, tuple(position), tuple(quaternion)))
        cutoff = now - self.gripper_args.stability_window_sec
        # 保留恰好跨过窗口起点的前一个样本，否则离散采样会使实际覆盖时长
        # 永远略小于配置窗口，导致稳定判定无法触发。
        while (len(self.stability_samples) > 1 and
               self.stability_samples[1][0] <= cutoff):
            self.stability_samples.popleft()
        if not self.stability_samples:
            return

        covered = now - self.stability_samples[0][0]
        if covered + 1e-9 < self.gripper_args.stability_window_sec:
            return

        samples = list(self.stability_samples)
        position_jitter_mm = max(
            math.dist(first[1], second[1]) * 1000.0
            for index, first in enumerate(samples)
            for second in samples[index:])
        angle_jitter_deg = max(
            angular_error_deg(first[2], second[2])
            for index, first in enumerate(samples)
            for second in samples[index:])
        stable = (
            position_jitter_mm <=
            self.gripper_args.stability_position_jitter_mm and
            angle_jitter_deg <=
            self.gripper_args.stability_angle_jitter_deg)
        position_error_mm = math.dist(position, self.target) * 1000.0
        angle_error_deg = angular_error_deg(quaternion, self.target_quaternion)
        stable = stable and (
            position_error_mm <= self.args.position_tolerance_mm and
            angle_error_deg <= self.args.angle_tolerance_deg)

        if now - self.last_stability_log_at >= 1.0:
            self.last_stability_log_at = now
            target_text = ' target_error=%.3fmm/%.3f°' % (
                position_error_mm, angle_error_deg)
            self.get_logger().info(
                'STABILITY phase=%s window=%.2fs jitter=%.3fmm/%.3f° '
                'limit=%.3fmm/%.3f°%s stable=%s' %
                (self.phase, covered, position_jitter_mm, angle_jitter_deg,
                 self.gripper_args.stability_position_jitter_mm,
                 self.gripper_args.stability_angle_jitter_deg,
                 target_text, str(stable).lower()))
        if stable:
            if self.phase == 'returning':
                self.finish(
                    0, 'PASS: 已返回 YAML 记录的 base_link -> left_tool 位姿；'
                    f'误差 {position_error_mm:.3f} mm / '
                    f'{angle_error_deg:.3f} deg')
                return
            self.finish(
                0, 'PASS: STABILITY left_tool 进入连续位姿稳定窗口；'
                f'窗口抖动 {position_jitter_mm:.3f} mm / '
                f'{angle_jitter_deg:.3f} deg')

    def tick(self):
        if self.exit_code is not None:
            return
        if self.phase == 'approach':
            super().tick()
            if (self.phase == 'approach' and self.args.execute and
                    self.target is not None and self.exit_code is None):
                self.publish_gripper(GRIPPER_OPEN, time.monotonic())
                self.maybe_accept_stable_plateau()
        elif self.phase == 'returning':
            self.publish_gripper(GRIPPER_CLOSED, time.monotonic())
            now = time.monotonic()
            if self.return_duration > 1e-9:
                progress = min(
                    1.0, max(0.0, (now - self.return_started_at) /
                             self.return_duration))
            else:
                progress = 1.0
            self.target = tuple(
                start + progress * (goal - start)
                for start, goal in zip(
                    self.return_start_position, self.return_goal_position))
            self.target_quaternion = self.interpolate_quaternion(
                self.return_start_quaternion, self.return_goal_quaternion,
                progress)
            if progress < 1.0:
                self.reset_stability_window()
            super().tick()
            if (progress >= 1.0 and self.phase == 'returning' and
                    self.exit_code is None):
                self.maybe_accept_stable_plateau()
        elif self.phase in ('base_moving', 'base_returning'):
            now = time.monotonic()
            if now - self.last_base_move_poll_at < 0.2:
                return
            self.last_base_move_poll_at = now
            try:
                state = get_json(
                    self.gripper_args.vlahost_url, '/base_move_world')
            except ValueError as error:
                self.get_logger().warning(f'等待底盘闭环状态失败：{error}')
                return
            move = state.get('move', {})
            status = move.get('status')
            if status == 'reached':
                self.base_move_started_at = None
                if self.phase == 'base_moving':
                    self.phase = 'opening'
                    self.open_started_at = now
                    self.get_logger().warning(
                        '底盘右移完成；开始将左夹爪从 1.0 缓慢松开到 0.0')
                else:
                    self.start_base_yaw_return('底盘世界 x/y 返回完成')
            elif status in ('failed', 'cancelled'):
                self.stop_base_motion()
                super().finish(
                    3, '底盘闭环%s未完成：%s；%s' %
                    ('右移' if self.phase == 'base_moving' else '反向返回',
                     status, move.get('reason', '未知原因')))
        elif self.phase == 'base_yaw_returning':
            self.base_yaw_return_tick()
        elif self.phase == 'opening':
            self.opening_tick()
        elif self.phase == 'go_home':
            self.go_home_tick()
        elif self.phase == 'lifting':
            self.lift_tick()
        elif self.phase == 'lifting_after_grasp':
            self.lift_after_grasp_tick()
        else:
            self.closing_tick()


def main():
    tag_args, motion, gripper = parse_args()
    rclpy.init(args=[])
    node = None
    try:
        return_pose_file, return_position, return_quaternion = load_return_pose(
            gripper.return_pose_file)
        node = TagCapture(tag_args)
        start = time.monotonic()
        while rclpy.ok() and node.goal is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - start > tag_args.capture_timeout:
                raise ValueError(
                    f'采集超时：未得到 ID {tag_args.tag_id} 的稳定观测')
        if node.goal is None:
            return 1
        goal = node.goal.copy()
        node.destroy_node()
        node = None
        motion.xyz = tuple(goal[:3, 3])
        node = TagGrasp(
            motion, goal, gripper, return_position, return_quaternion,
            return_pose_file)
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.exit_code if node.exit_code is not None else 1
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(f'apriltag_left_tool_grasp_test: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            if hasattr(node, 'stop_base_motion'):
                node.stop_base_motion()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
