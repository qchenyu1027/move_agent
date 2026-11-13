#!/usr/bin/env python3
# 用途：按桌面安全高度执行头部视觉抓取的分段航路。
# 预览：source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh && /usr/bin/python3 -B -m path_planner.safe_grasp_planner --table-z 0.70 --object-position 0.40 0.20 0.90 --current-position 0.30 0.15 0.85
# 实机：source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh && PYTHONNOUSERSITE=1 /usr/bin/python3 -B -m path_planner.safe_grasp_planner --table-z 0.70 --control-backend joint --joint-source 2 --object-pose-topic /object_pose --execute
"""分段抓取路径规划器。

程序先抬高手臂，再水平移动到预抓取点上方，到预抓取点后闭合夹爪，
然后从该点垂直抬升。每个执行目标都等待实际工具位姿到位；任一 TF、IK 话题或误差
检查失败，立即发布空目标并停止。
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import time

from .core import (PlanError, Waypoint, build_grasp_plan, densify_waypoints,
                   distance, held_joint_configuration)


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'tools' / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

try:
    import numpy as np
    # ROS Humble 的 Pinocchio 二进制通常按 NumPy 1.x 编译；NumPy 2.x
    # 导入该扩展可能直接触发解释器崩溃，因此必须在 import pinocchio 前拦截。
    numpy_major = int(np.__version__.split('.', 1)[0])
    if numpy_major >= 2:
        pin = None
        PIN_ERROR = RuntimeError(
            f'当前 NumPy {np.__version__} 不兼容 ROS Pinocchio，请设置 PYTHONNOUSERSITE=1')
    else:
        try:
            import pinocchio as pin
            PIN_ERROR = None
        except Exception as error:  # NumPy/Pinocchio ABI 错误也要转成可读提示。
            pin = None
            PIN_ERROR = error
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped
    from marvin_msgs.msg import IKRequest, JointcmdArm, JointcmdBody, JointcmdHead, Jointfeedback
    from marvin_msgs.srv import Int
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from std_msgs.msg import Bool, Float32, Int32
    from tf2_ros import Buffer, TransformException, TransformListener
    from left_tool_accuracy_test import (
        angular_error_deg, multiply_quaternions, read_tool_mount,
        rotate_vector, topic_name, transform_pose,
    )
except ImportError:  # 预览核心规划时允许没有 ROS 环境。
    rclpy = None
    np = None
    pin = None
    PIN_ERROR = None


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right'), default='left')
    parser.add_argument('--execute', action='store_true', help='发布 IK 和夹爪控制；默认只预览')
    parser.add_argument('--object-pose-topic', default='/object_pose')
    parser.add_argument('--object-pose-change-topic', default='/object_pose_changed')
    parser.add_argument('--object-position', nargs=3, type=float,
                        metavar=('X', 'Y', 'Z'), help='预览用 base_link 物体位置，单位 m')
    parser.add_argument('--current-position', nargs=3, type=float,
                        metavar=('X', 'Y', 'Z'), help='预览用 base_link 工具位置，单位 m')
    parser.add_argument('--table-z', type=float, required=True,
                        help='base_link 坐标系中的桌面高度，单位 m')
    parser.add_argument('--table-clearance-mm', type=float, default=100.0)
    parser.add_argument('--start-clearance-mm', type=float, default=20.0,
                        help='当前工具点距桌面的最小安全余量；第一段随后只能垂直抬高')
    parser.add_argument('--grasp-offset-mm', nargs=3, type=float, default=(0.0, 0.0, -40.0))
    parser.add_argument('--approach-front-mm', type=float, default=80.0)
    parser.add_argument('--approach-height-mm', type=float, default=30.0)
    parser.add_argument('--overhead-mm', type=float, default=80.0)
    parser.add_argument('--lift-height-mm', type=float, default=150.0)
    parser.add_argument('--max-step-mm', type=float, default=10.0)
    parser.add_argument('--position-tolerance-mm', type=float, default=10.0)
    parser.add_argument('--angle-tolerance-deg', type=float, default=5.0)
    parser.add_argument('--settle-time', type=float, default=0.4)
    parser.add_argument('--stable-samples', type=int, default=5)
    parser.add_argument('--stable-position-mm', type=float, default=5.0)
    parser.add_argument('--capture-timeout', type=float, default=45.0)
    parser.add_argument('--motion-timeout', type=float, default=30.0)
    parser.add_argument('--namespace', default='/tj')
    parser.add_argument('--control-backend', choices=('joint', 'ros'), default='joint',
                        help='joint=本地单臂 IK/user joint；ros=发布 ROS IK')
    parser.add_argument('--ik-topic', default='control/ik_request/vla')
    parser.add_argument('--ik-source', type=int, default=2)
    parser.add_argument('--joint-source', type=int, default=None,
                        help='关节命令源；joint 默认 2，ros 默认 0')
    parser.add_argument('--max-joint-speed-rad-s', type=float, default=0.3)
    parser.add_argument('--max-body-deviation-deg', type=float, default=5.0,
                        help='躯干任一关节偏离接管时姿态的停止阈值，默认 5°')
    parser.add_argument('--model-xml')
    parser.add_argument('--gripper-topic')
    parser.add_argument('--close-position', type=float, default=0.7)
    parser.add_argument('--output', type=Path, help='预览时保存航路 JSON')
    args = parser.parse_args(argv)
    positive = (
        'table_clearance_mm', 'approach_front_mm', 'approach_height_mm',
        'overhead_mm', 'lift_height_mm', 'max_step_mm',
        'position_tolerance_mm', 'angle_tolerance_deg', 'settle_time',
        'capture_timeout', 'motion_timeout', 'stable_position_mm',
    )
    for name in positive:
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f'--{name.replace("_", "-")} 必须为正数')
    if args.stable_samples < 2:
        parser.error('--stable-samples 至少为 2')
    if args.joint_source is None:
        args.joint_source = 2 if args.control_backend == 'joint' else 0
    if args.ik_source < 0 or args.joint_source < 0:
        parser.error('--ik-source 和 --joint-source 必须是非负整数')
    if not math.isfinite(args.max_joint_speed_rad_s) or args.max_joint_speed_rad_s <= 0.0:
        parser.error('--max-joint-speed-rad-s 必须为正数')
    if not math.isfinite(args.max_body_deviation_deg) or args.max_body_deviation_deg <= 0.0:
        parser.error('--max-body-deviation-deg 必须为正数')
    if not 0.0 <= args.close_position <= 1.0:
        parser.error('--close-position 必须在 0..1 之间')
    if args.object_position is not None and args.current_position is None:
        parser.error('使用 --object-position 时必须同时提供 --current-position')
    if args.execute and args.object_position is not None:
        parser.error('--execute 必须从 ROS --object-pose-topic 获取实时目标')
    return args


def waypoint_json(waypoints: list[Waypoint]):
    return [
        {'name': point.name,
         'position_m': list(point.position),
         'quaternion_xyzw': list(point.quaternion)}
        for point in waypoints
    ]


def preview(args):
    current = tuple(args.current_position)
    object_position = tuple(args.object_position)
    waypoints = build_grasp_plan(
        current, (0.0, 0.0, 0.0, 1.0), object_position,
        grasp_offset_mm=args.grasp_offset_mm,
        approach_front_mm=args.approach_front_mm,
        approach_height_mm=args.approach_height_mm,
        overhead_mm=args.overhead_mm,
        lift_height_mm=args.lift_height_mm,
        table_z_m=args.table_z,
        table_clearance_mm=args.table_clearance_mm,
        start_clearance_mm=args.start_clearance_mm,
        max_step_mm=args.max_step_mm,
    )
    payload = waypoint_json(waypoints)
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    if args.output:
        args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print(f'已保存航路：{args.output}')


if rclpy is not None:
    class SafeGraspPlanner(Node):
        """接收视觉目标并逐航路点发布 ROS IK。"""

        def __init__(self, args):
            super().__init__('safe_grasp_path_planner')
            self.args = args
            self.side_index = 0 if args.side == 'left' else 1
            self.arm_link = 'Arm_L7_Link' if self.side_index == 0 else 'Arm_R7_Link'
            self.joint_positions = None
            self.joint_feedback_at = 0.0
            self.user_joint_hold = None
            self.user_joint_target = None
            self.joint_seed_started = None
            model_xml = args.model_xml or str(Path(
                get_package_share_directory('marvin_description')) /
                'mjcf/Gento_Luna/Gento_Luna.xml')
            if args.control_backend == 'joint':
                if pin is None or np is None:
                    raise RuntimeError(
                        'joint 后端需要可用的 Pinocchio/NumPy；请使用 PYTHONNOUSERSITE=1，'
                        f'当前导入错误：{PIN_ERROR}')
                self.ik_model = pin.buildModelFromMJCF(model_xml)
                if self.ik_model.nq != 22 or self.ik_model.nv != 22:
                    raise RuntimeError(
                        f'MJCF 关节数需为 22，实际 nq={self.ik_model.nq}, nv={self.ik_model.nv}')
                self.ik_data = self.ik_model.createData()
                self.ik_frame_id = self.ik_model.getFrameId(
                    'left_tool' if self.side_index == 0 else 'right_tool')
                if self.ik_frame_id >= len(self.ik_model.frames):
                    raise RuntimeError('MJCF 缺少所选手臂工具 frame')
            else:
                self.ik_model = None
                self.ik_data = None
                self.ik_frame_id = None
            self.mount_position, self.mount_quaternion = read_tool_mount(
                model_xml, 'L' if self.side_index == 0 else 'R')
            self.object_topic = args.object_pose_topic
            self.ik_topic = topic_name(args.namespace, args.ik_topic)
            self.gripper_topic = args.gripper_topic or topic_name(
                args.namespace, f'control/gripperValue{"L" if self.side_index == 0 else "R"}')
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.ik_publisher = (self.create_publisher(IKRequest, self.ik_topic, qos)
                                 if args.control_backend == 'ros' else None)
            self.user_joint_publishers = ()
            if args.control_backend == 'joint':
                self.user_joint_publishers = (
                    self.create_publisher(JointcmdArm,
                                          topic_name(args.namespace, 'control/user/joint_cmd_A'), qos),
                    self.create_publisher(JointcmdArm,
                                          topic_name(args.namespace, 'control/user/joint_cmd_B'), qos),
                    self.create_publisher(JointcmdBody,
                                          topic_name(args.namespace, 'control/user/joint_cmd_body'), qos),
                    self.create_publisher(JointcmdHead,
                                          topic_name(args.namespace, 'control/user/joint_cmd_head'), qos),
                )
                self.create_subscription(
                    Jointfeedback,
                    topic_name(args.namespace, 'info/joint_feedback'),
                    self.on_joint_feedback,
                    qos_profile_sensor_data)
            self.gripper_publisher = self.create_publisher(Float32, self.gripper_topic, 10)
            self.select_client = self.create_client(
                Int, topic_name(args.namespace, 'control/ik_request_mux/select'))
            self.joint_select_client = self.create_client(
                Int, topic_name(args.namespace, 'control/joint_cmd_mux/select'))
            self.create_subscription(PoseStamped, self.object_topic, self.on_pose,
                                     qos_profile_sensor_data)
            if args.object_pose_change_topic:
                self.create_subscription(Bool, args.object_pose_change_topic,
                                         self.on_pose_change, 10)
            self.create_subscription(Int32,
                                     topic_name(args.namespace, 'info/ik_request_mux/active_source'),
                                     lambda message: setattr(self, 'ik_source', message.data), 10)
            self.joint_source = None
            self.create_subscription(
                Int32,
                topic_name(args.namespace, 'info/joint_cmd_mux/active_source'),
                lambda message: setattr(self, 'joint_source', message.data), 10)
            self.pending_pose = None
            self.samples = deque(maxlen=args.stable_samples)
            self.last_stamp = None
            self.ik_source = None
            self.plan: list[Waypoint] = []
            self.phase = 'capture'
            self.phase_started = time.monotonic()
            self.started = self.phase_started
            self.waypoint_index = 1
            self.settled_since = None
            self.last_motion_log = 0.0
            self.change_pending = False
            self.exit_code = None
            self.timer = self.create_timer(0.05, self.tick)
            self.get_logger().info(
                f'等待 {self.object_topic}；航路为抬高→平移→预抓取点闭合→抬升；'
                f'{"执行" if args.execute else "预览"}模式；'
                f'控制后端={"本地单臂 IK/user joint" if args.control_backend == "joint" else "ROS IK"}')

        def on_joint_feedback(self, message):
            positions = [*message.body_positions[:6], *message.head_positions[:2],
                         *message.arm_positions[:14]]
            if len(positions) == 22 and all(math.isfinite(value) for value in positions):
                self.joint_positions = [float(value) for value in positions]
                self.joint_feedback_at = time.monotonic()

        def on_pose(self, message):
            if self.phase == 'capture':
                self.pending_pose = message

        def on_pose_change(self, message):
            lifting_object = (
                self.phase == 'move' and self.plan and
                self.plan[self.waypoint_index].name.startswith('lift_after_grasp'))
            if message.data and self.phase not in ('capture', 'closing', 'done') and not lifting_object:
                self.change_pending = True

        def finish(self, code, reason):
            if self.exit_code is not None:
                return
            self.exit_code = code
            self.publish_unavailable()
            self.get_logger().info(reason) if code == 0 else self.get_logger().error(reason)

        def publish_unavailable(self):
            if self.args.control_backend == 'joint':
                if self.user_joint_hold is not None and self.joint_positions is not None:
                    hold = held_joint_configuration(
                        self.user_joint_hold, self.joint_positions, self.args.side)
                    self.publish_user_joint(hold)
                return
            message = IKRequest()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'base_link'
            message.available = [False] * 11
            self.ik_publisher.publish(message)

        def object_in_base(self, message):
            frame = message.header.frame_id.strip('/')
            if not frame:
                raise ValueError('物体位姿 frame_id 为空')
            transform = self.buffer.lookup_transform(
                'base_link', frame, Time(), timeout=Duration(seconds=0.05))
            origin, rotation = transform_pose(transform)
            p = message.pose.position
            position = tuple(a + b for a, b in zip(origin, rotate_vector(
                (p.x, p.y, p.z), rotation)))
            if not all(math.isfinite(value) for value in position):
                raise ValueError('物体位置包含非有限数值')
            return position

        def actual_tool(self):
            transform = self.buffer.lookup_transform(
                'base_link', self.arm_link, Time(), timeout=Duration(seconds=0.05))
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if stamp <= 0.0 or not -0.05 <= age <= 0.5:
                raise ValueError(f'{self.arm_link} TF 已过期：{age:.3f} s')
            position, quaternion = transform_pose(transform)
            return (
                tuple(a + b for a, b in zip(position,
                                            rotate_vector(self.mount_position, quaternion))),
                multiply_quaternions(quaternion, self.mount_quaternion),
            )

        def capture(self, now):
            message = self.pending_pose
            if message is None:
                return
            self.pending_pose = None
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if stamp <= 0.0 or not -0.05 <= age <= 0.5 or stamp == self.last_stamp:
                return
            self.last_stamp = stamp
            try:
                position = self.object_in_base(message)
            except (ValueError, TransformException) as error:
                self.samples.clear()
                self.get_logger().warning(f'等待有效物体位姿：{error}', throttle_duration_sec=1.0)
                return
            if self.samples and distance(self.samples[-1], position) * 1000.0 > self.args.stable_position_mm:
                self.samples.clear()
            self.samples.append(position)
            if len(self.samples) < self.args.stable_samples:
                return
            reference = self.samples[-1]
            if any(distance(sample, reference) * 1000.0 > self.args.stable_position_mm
                   for sample in self.samples):
                return
            try:
                actual, quaternion = self.actual_tool()
                key_plan = build_grasp_plan(
                    actual, quaternion, reference,
                    grasp_offset_mm=self.args.grasp_offset_mm,
                    approach_front_mm=self.args.approach_front_mm,
                    approach_height_mm=self.args.approach_height_mm,
                    overhead_mm=self.args.overhead_mm,
                    lift_height_mm=self.args.lift_height_mm,
                    table_z_m=self.args.table_z,
                    table_clearance_mm=self.args.table_clearance_mm,
                    start_clearance_mm=self.args.start_clearance_mm,
                    max_step_mm=self.args.max_step_mm,
                )
                self.plan = densify_waypoints(
                    key_plan, self.args.max_step_mm / 1000.0,
                    direct_targets=('pregrasp', 'lift_after_grasp'))
            except (PlanError, ValueError, TransformException) as error:
                self.finish(1, f'安全航路规划失败：{error}')
                return
            for point in key_plan:
                self.get_logger().info(
                    f'航路点 {point.name}: position={point.position}')
            self.get_logger().info(
                f'执行目标数：{len(self.plan)}；平移阶段最大步长={self.args.max_step_mm:g} mm；'
                '预抓取点到位后闭合夹爪，再从该点直接抬升')
            if not self.args.execute:
                self.finish(0, '预览完成；未发布机器人运动')
                return
            self.phase = 'preflight'
            self.phase_started = now
            self.waypoint_index = 1

        def select_source(self, now):
            if self.args.control_backend == 'joint':
                current = self.feedback_configuration()
                if self.joint_seed_started is None:
                    self.joint_seed_started = now
                    self.get_logger().info(
                        '接管 user joint 前发送当前 22 关节保持目标')
                if (now - self.joint_seed_started < 0.5 or
                        self.joint_source != self.args.joint_source):
                    # 接管前跟随最新实测值预热缓存；确认接管后才锁定躯干目标。
                    self.user_joint_hold = current
                    self.user_joint_target = list(current)
                    self.publish_user_joint(current)
                if now - self.joint_seed_started < 0.5:
                    return False
                if self.joint_source == self.args.joint_source:
                    self.get_logger().info(
                        f'已固定接管时躯干/另一侧手臂目标；躯干={self.user_joint_hold[:6]}')
                    return True
                if not self.joint_select_client.service_is_ready():
                    return False
                if not hasattr(self, 'joint_select_future'):
                    request = Int.Request()
                    request.data = self.args.joint_source
                    self.joint_select_future = self.joint_select_client.call_async(request)
                    return False
                if not self.joint_select_future.done():
                    return False
                response = self.joint_select_future.result()
                if (response is None or not response.success or
                        response.error_code != self.args.joint_source):
                    raise RuntimeError(
                        f'关节 mux source 切换失败：{getattr(response, "message", "无响应")}')
                return self.joint_source == self.args.joint_source

            if (self.ik_source == self.args.ik_source and
                    self.joint_source == self.args.joint_source):
                return True
            if (not self.select_client.service_is_ready() or
                    not self.joint_select_client.service_is_ready()):
                return False
            if not hasattr(self, 'joint_select_future'):
                request = Int.Request()
                request.data = self.args.joint_source
                self.joint_select_future = self.joint_select_client.call_async(request)
                return False
            if not self.joint_select_future.done():
                return False
            response = self.joint_select_future.result()
            if (response is None or not response.success or
                    response.error_code != self.args.joint_source):
                raise RuntimeError(f'关节 mux source 切换失败：{getattr(response, "message", "无响应")}')
            if not hasattr(self, 'ik_select_future'):
                request = Int.Request()
                request.data = self.args.ik_source
                self.ik_select_future = self.select_client.call_async(request)
                return False
            if not self.ik_select_future.done():
                return False
            response = self.ik_select_future.result()
            if (response is None or not response.success or
                    response.error_code != self.args.ik_source):
                raise RuntimeError(f'IK mux source 切换失败：{getattr(response, "message", "无响应")}')
            return (self.ik_source == self.args.ik_source and
                    self.joint_source == self.args.joint_source)

        def feedback_configuration(self):
            if (self.joint_positions is None or
                    time.monotonic() - self.joint_feedback_at > 0.5):
                raise ValueError('22 关节反馈缺失或过期')
            return list(self.joint_positions)

        def publish_user_joint(self, positions):
            if len(positions) != 22 or not all(math.isfinite(value) for value in positions):
                raise ValueError('user joint 命令必须包含 22 个有限关节角')
            stamp = self.get_clock().now().to_msg()
            left = JointcmdArm(); left.header.stamp = stamp; left.positions = positions[8:15]
            right = JointcmdArm(); right.header.stamp = stamp; right.positions = positions[15:22]
            body = JointcmdBody(); body.header.stamp = stamp; body.positions = positions[:6]
            head = JointcmdHead(); head.header.stamp = stamp; head.positions = [positions[6], positions[7], 0.0]
            for publisher, message in zip(self.user_joint_publishers, (left, right, body, head)):
                publisher.publish(message)

        def solve_arm_ik(self, q, position, quaternion):
            start = 8 if self.side_index == 0 else 15
            indices = list(range(start, start + 7))
            target = pin.SE3(
                pin.Quaternion(float(quaternion[3]), float(quaternion[0]),
                               float(quaternion[1]), float(quaternion[2])).toRotationMatrix(),
                np.asarray(position, dtype=float))
            last_position_error = float('inf')
            last_angle_error = float('inf')
            for use_position_ik in (False, True):
                solution = np.asarray(q, dtype=float).copy()
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
                        self.ik_model, self.ik_data, solution, self.ik_frame_id,
                        reference)[:rows, indices]
                    velocity = jacobian.T @ np.linalg.solve(
                        jacobian @ jacobian.T + 1e-4 * np.eye(rows), error)
                    norm = float(np.linalg.norm(velocity))
                    if norm > 0.12:
                        velocity *= 0.12 / norm
                    delta = np.zeros(self.ik_model.nv)
                    delta[indices] = velocity
                    solution = pin.integrate(self.ik_model, solution, delta * 0.7)
                    for index in indices:
                        solution[index] = min(
                            max(solution[index], self.ik_model.lowerPositionLimit[index]),
                            self.ik_model.upperPositionLimit[index])
                pin.forwardKinematics(self.ik_model, self.ik_data, solution)
                pin.updateFramePlacements(self.ik_model, self.ik_data)
                final = self.ik_data.oMf[self.ik_frame_id]
                last_position_error = float(
                    np.linalg.norm(target.translation - final.translation) * 1000.0)
                last_angle_error = math.degrees(float(
                    np.linalg.norm(pin.log3(final.rotation.T @ target.rotation))))
                if (last_position_error <= self.args.position_tolerance_mm and
                        last_angle_error <= self.args.angle_tolerance_deg):
                    result = list(q)
                    result[start:start + 7] = solution[start:start + 7].tolist()
                    return result
            raise RuntimeError(
                f'单臂位置 IK 未收敛：位置误差={last_position_error:.1f} mm，'
                f'姿态误差={last_angle_error:.1f}°')

        def make_user_joint_target(self, point, actual, actual_quaternion):
            if self.user_joint_hold is None:
                raise ValueError('尚未建立关节保持目标')
            q = held_joint_configuration(
                self.user_joint_hold, self.feedback_configuration(), self.args.side)
            pin.forwardKinematics(self.ik_model, self.ik_data, np.asarray(q))
            pin.updateFramePlacements(self.ik_model, self.ik_data)
            model_pose = self.ik_data.oMf[self.ik_frame_id]
            model_position = tuple(float(value) for value in model_pose.translation)
            model_quaternion = tuple(float(value) for value in
                                    pin.Quaternion(model_pose.rotation).coeffs())
            inverse_actual = (-actual_quaternion[0], -actual_quaternion[1],
                              -actual_quaternion[2], actual_quaternion[3])
            delta_quaternion = multiply_quaternions(point.quaternion, inverse_actual)
            target_position = tuple(
                float(model + target - observed)
                for model, target, observed in zip(model_position, point.position, actual))
            target_quaternion = multiply_quaternions(delta_quaternion, model_quaternion)
            return self.solve_arm_ik(q, target_position, target_quaternion)

        def publish_target(self, point):
            actual, quaternion = self.actual_tool()
            if self.args.control_backend == 'joint':
                if self.joint_source != self.args.joint_source:
                    raise ValueError(f'关节 mux 已切离 source {self.args.joint_source}')
                current = self.feedback_configuration()
                deviation = max(
                    abs(actual - target)
                    for actual, target in zip(current[:6], self.user_joint_hold[:6]))
                if math.degrees(deviation) > self.args.max_body_deviation_deg:
                    raise ValueError(
                        f'躯干关节偏离接管姿态 {math.degrees(deviation):.1f}°，'
                        '停止抓取并保持初始躯干目标')
                goal = self.make_user_joint_target(point, actual, quaternion)
                if self.user_joint_target is None:
                    raise ValueError('尚未建立关节速度限制的初始目标')
                max_step = self.args.max_joint_speed_rad_s * 0.05
                for index in range(8 if self.side_index == 0 else 15,
                                   (15 if self.side_index == 0 else 22)):
                    change = goal[index] - self.user_joint_target[index]
                    goal[index] = self.user_joint_target[index] + max(
                        -max_step, min(change, max_step))
                self.publish_user_joint(goal)
                self.user_joint_target = goal
                return
            message = IKRequest()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'base_link'
            message.available = [False] * 11
            message.available[self.side_index] = True
            pose = message.left_hand_pose if self.side_index == 0 else message.right_hand_pose
            pose.position.x, pose.position.y, pose.position.z = point.position
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = point.quaternion
            self.ik_publisher.publish(message)

        def tick_move(self, now):
            point = self.plan[self.waypoint_index]
            try:
                actual, quaternion = self.actual_tool()
                if point.name in ('pregrasp', 'lift_after_grasp'):
                    minimum_z = self.args.table_z + self.args.table_clearance_mm / 1000.0
                    if actual[2] < minimum_z:
                        raise ValueError(
                            f'实际工具 Z={actual[2]:.3f} m 低于桌面安全高度 '
                            f'{minimum_z:.3f} m')
                self.publish_target(point)
            except (ValueError, TransformException) as error:
                self.finish(1, f'运动期间安全检查失败：{error}')
                return
            except RuntimeError as error:
                self.finish(1, f'运动目标生成失败：{error}')
                return
            position_error = distance(actual, point.position) * 1000.0
            angle_error = angular_error_deg(quaternion, point.quaternion)
            if position_error <= self.args.position_tolerance_mm and angle_error <= self.args.angle_tolerance_deg:
                if self.settled_since is None:
                    self.settled_since = now
                elif now - self.settled_since >= self.args.settle_time:
                    self.get_logger().info(
                        f'航路点 {point.name} 到位：位置误差={position_error:.1f} mm，'
                        f'姿态误差={angle_error:.1f}°')
                    if point.name == 'pregrasp':
                        self.phase = 'closing'
                        self.phase_started = now
                        self.get_logger().info('预抓取点到位，先闭合夹爪再从该点抬升')
                    elif self.waypoint_index >= len(self.plan) - 1:
                        self.finish(0, '夹爪闭合命令已发送，抬升目标到位；未验证是否夹住物体')
                    else:
                        self.waypoint_index += 1
                        self.settled_since = None
                        self.phase_started = now
                        self.last_motion_log = 0.0
            else:
                self.settled_since = None
                if now - self.last_motion_log >= 1.0:
                    self.get_logger().info(
                        f'执行 {point.name}：位置误差={position_error:.1f} mm，'
                        f'姿态误差={angle_error:.1f}°；'
                        f'目标={point.position}')
                    self.last_motion_log = now
            if now - self.phase_started > self.args.motion_timeout:
                self.finish(2, f'航路点 {point.name} 超时：位置误差={position_error:.1f} mm')

        def tick(self):
            if self.exit_code is not None:
                return
            now = time.monotonic()
            if self.change_pending:
                self.change_pending = False
                self.publish_unavailable()
                self.phase = 'capture'
                self.phase_started = now
                self.started = now
                self.samples.clear()
                self.pending_pose = None
                self.plan = []
                self.get_logger().warning('物体位置变化，停止当前航路并重新采集')
                return
            if self.phase == 'capture':
                if (self.args.control_backend == 'joint' and
                        self.user_joint_hold is not None and
                        self.joint_source == self.args.joint_source):
                    try:
                        self.publish_unavailable()
                    except (ValueError, PlanError) as error:
                        self.finish(1, f'采集期间保持关节目标失败：{error}')
                        return
                self.capture(now)
                if self.phase == 'capture' and now - self.started > self.args.capture_timeout:
                    self.finish(1, '等待稳定物体位姿超时')
            elif self.phase == 'preflight':
                if self.args.control_backend == 'ros':
                    control_ready = self.ik_publisher.get_subscription_count() > 0
                else:
                    control_ready = all(
                        publisher.get_subscription_count() > 0
                        for publisher in self.user_joint_publishers)
                if not control_ready:
                    if now - self.phase_started > self.args.motion_timeout:
                        if self.args.control_backend == 'ros':
                            reason = f'{self.ik_topic} 没有订阅者'
                        else:
                            reason = 'user joint 命令话题没有订阅者'
                        self.finish(1, reason)
                    return
                try:
                    if self.select_source(now):
                        self.phase = 'move'
                        self.phase_started = now
                        self.get_logger().info(
                            f'关节 mux={self.joint_source}、IK mux={self.ik_source} 已就绪，'
                            '开始执行分段航路')
                except (RuntimeError, ValueError) as error:
                    self.finish(1, str(error))
                if self.phase == 'preflight' and now - self.phase_started > self.args.motion_timeout:
                    self.finish(1, '等待关节/IK mux 切换超时；检查 select 服务和控制器状态')
            elif self.phase == 'move':
                self.tick_move(now)
            elif self.phase == 'closing':
                self.gripper_publisher.publish(Float32(data=float(self.args.close_position)))
                if now - self.phase_started >= 1.0:
                    self.phase = 'move'
                    self.waypoint_index += 1
                    self.phase_started = now
                    self.settled_since = None
                    self.last_motion_log = 0.0
                    self.get_logger().info('夹爪闭合命令已发送，开始抬升')


def main(argv=None):
    args = parse_args(argv)
    if args.object_position is not None:
        preview(args)
        return 0
    if rclpy is None:
        print('错误：ROS 环境未加载，请先 source apex_ros_env.sh。', file=sys.stderr)
        return 2
    rclpy.init(args=[])
    node = None
    try:
        node = SafeGraspPlanner(args)
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.exit_code if node.exit_code is not None else 1
    except RuntimeError as error:
        print(f'错误：{error}', file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
