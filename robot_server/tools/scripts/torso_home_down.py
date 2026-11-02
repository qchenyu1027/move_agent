#!/usr/bin/env python3
# 用途：先到独立保存的原始双手水平 home，再按目标毫米数升降躯干；运行：python3 -B -m agent_tools.scheduler agent_tools/task.json --execute。
"""先到原始双手水平 home，再只联动躯干完成高度偏移。

``--offset-mm`` 为有符号值：负数表示下降，正数表示上升。工具不修改当前
MJCF 的 keyframe/home，而是加载 tools/scripts/luna_horizontal_home.json 作为
动作内的基准姿态。高度 IK 只使用 6 个 Leg_J 关节，双臂保持原始 home
关节角，不再为保持末端世界位姿而补偿；高度 IK 放开躯干前后（X）位置，
只保持左右（Y）、高度（Z）和躯干姿态；头部不参与到位判定，但按 home
文件中的目标角度发送。
"""

import argparse
import json
import math
import os
import time
import xml.etree.ElementTree as ET

import numpy as np


DEFAULT_MODEL = '/home/liuqian/WBC/Apex_Deploy/robot_node/marvin_description/mjcf/Gento_Luna/Gento_Luna.xml'
DEFAULT_HOME = os.path.join(os.path.dirname(__file__), 'luna_horizontal_home.json')
DEFAULT_MAX_OFFSET_MM = float('inf')  # 默认不设人为上限；偏移幅度由 IK 收敛和关节限位约束
# Leg_J4 的 MJCF 运动学 range 上限(0.71)比真实限位保守；URDF <limit upper> 与执行器
# ctrlrange 一致认定真实上限为 0.8 rad。IK 对齐到真实值，可把水平·原地下降从约 -180mm
# 扩到约 -234mm（仍受 Leg_J4 这个真实限位约束）。如需恢复保守值，把此常量改回 0.71。
LEG_J4_REAL_UPPER = 0.8
BODY_NAMES = [f'Leg_J{i}_Joint' for i in range(1, 7)]
HEAD_NAMES = ['Head_J1_joint', 'Head_J2_Joint']
ARM_NAMES = [f'Arm_{side}{i}_Joint' for side in ('L', 'R') for i in range(1, 8)]
TORSO_INDICES = list(range(6))
VARIABLE_INDICES = TORSO_INDICES
CONTROLLED_INDICES = TORSO_INDICES + list(range(8, 22))
FEEDBACK_MAX_AGE_S = 1.0


def _transform(pos, quat):
    w, x, y, z = np.asarray(quat, dtype=float)
    norm = np.linalg.norm([w, x, y, z])
    if norm <= 1e-12:
        raise ValueError('MJCF 中存在零四元数')
    w, x, y, z = np.asarray([w, x, y, z]) / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]
    matrix[:3, 3] = pos
    return matrix


def _load_model(model_path):
    root = ET.parse(model_path).getroot()
    joints = root.findall('./worldbody//joint')
    names = [joint.get('name') for joint in joints]
    expected = BODY_NAMES + HEAD_NAMES + ARM_NAMES
    if names != expected:
        raise ValueError('MJCF 关节顺序与 Luna 原始 home 记录不一致')
    lower = np.asarray([float(joint.get('range').split()[0]) for joint in joints])
    upper = np.asarray([float(joint.get('range').split()[1]) for joint in joints])
    # 仅放宽 Leg_J4：MJCF 的运动学 range 上限 0.71 rad 比真实执行范围保守。
    # URDF 的 <limit upper="0.8"> 与该关节执行器 ctrlrange 上限 0.8 rad 一致，
    # 因此 IK 使用 0.8；其它关节仍严格使用 MJCF range，避免扩大安全边界。
    leg_j4_index = names.index('Leg_J4_Joint')
    upper[leg_j4_index] = LEG_J4_REAL_UPPER
    return root, names, lower, upper


def _forward_kinematics(root, joint_names, q):
    angles = dict(zip(joint_names, q))
    poses = {}

    def walk(body, parent):
        current = parent @ _transform(
            [float(value) for value in body.get('pos', '0 0 0').split()],
            [float(value) for value in body.get('quat', '1 0 0 0').split()],
        )
        for joint in body.findall('joint'):
            axis = np.asarray(
                [float(value) for value in joint.get('axis', '0 0 1').split()],
                dtype=float,
            )
            axis /= np.linalg.norm(axis)
            angle = angles[joint.get('name')]
            current = current @ _transform(
                [0, 0, 0], [math.cos(angle / 2.0), *(math.sin(angle / 2.0) * axis)])
        if body.get('name'):
            poses[body.get('name')] = current
        for child in body.findall('body'):
            walk(child, current)

    for body in root.findall('./worldbody/body'):
        walk(body, np.eye(4))
    return poses


def _rotation_error(current, target):
    relative = target.T @ current
    return 0.5 * np.asarray([
        relative[2, 1] - relative[1, 2],
        relative[0, 2] - relative[2, 0],
        relative[1, 0] - relative[0, 1],
    ])


def _task_residual(poses, targets):
    residual = []
    for name in ('left_tool', 'right_tool'):
        residual.extend(targets[name][:3, 3] - poses[name][:3, 3])
        residual.extend(_rotation_error(poses[name][:3, :3], targets[name][:3, :3]))
    residual.extend(targets['torso_link'][:3, 3] - poses['torso_link'][:3, 3])
    return np.asarray(residual, dtype=float)


def _torso_residual(pose, target):
    """约束躯干 Y、Z 和姿态，放开 X 前后位置。

    X 不进入残差，因此求解器可以通过前后移动换取更大的 Z 高度范围；
    正则项仍会让解尽量接近上一步姿态。双臂关节始终由调用方保持 home。
    """
    return np.concatenate([
        target[1:3, 3] - pose[1:3, 3],
        _rotation_error(pose[:3, :3], target[:3, :3]),
    ])


def _solve_torso_to_target(root, joint_names, lower, upper, seed_q, torso_target):
    """只动 6 个躯干关节，从 seed_q 做阻尼牛顿迭代逼近 torso_target；返回解和最大残差。"""
    q = np.asarray(seed_q, dtype=float).copy()
    regularization = 0.01
    for _ in range(120):
        poses = _forward_kinematics(root, joint_names, q)
        residual = _torso_residual(poses['torso_link'], torso_target)
        if np.linalg.norm(residual) < 1e-7:
            break
        jacobian = np.zeros((residual.size, len(VARIABLE_INDICES)))
        for column, index in enumerate(VARIABLE_INDICES):
            perturbed = q.copy()
            perturbed[index] += 1e-6
            perturbed_residual = _torso_residual(
                _forward_kinematics(root, joint_names, perturbed)['torso_link'], torso_target)
            jacobian[:, column] = (perturbed_residual - residual) / 1e-6
        matrix = np.vstack([jacobian, regularization * np.eye(len(VARIABLE_INDICES))])
        rhs = np.concatenate([-residual, np.zeros(len(VARIABLE_INDICES))])
        delta = np.linalg.lstsq(matrix, rhs, rcond=None)[0]
        step_scale = max(1.0, np.linalg.norm(delta) / 0.12)
        q[VARIABLE_INDICES] = np.clip(
            q[VARIABLE_INDICES] + delta / step_scale,
            lower[VARIABLE_INDICES], upper[VARIABLE_INDICES])
    final_residual = _torso_residual(
        _forward_kinematics(root, joint_names, q)['torso_link'], torso_target)
    return q, float(np.max(np.abs(final_residual)))


def solve_height_offset(model_path, home_q, offset_mm, step_mm=5.0):
    """只调整 6 个躯干关节，双臂关节保持原始 home 值。

    采用连续化/热启动：从 0 开始按 step_mm 逼近目标偏移，每小步用上一步的解做
    初值。单步线性初值在大偏移（约 -106mm 外）会让牛顿迭代发散。保持 X 前后
    位置时下限约 -234mm（受放宽后的 Leg_J4=0.8 rad 限位约束）；本工具放开 X
    后可通过前后位移继续下降，直到其它关节约束或 IK 不再收敛。
    """
    root, joint_names, lower, upper = _load_model(model_path)
    home = np.asarray(home_q, dtype=float)
    if home.shape != (22,):
        raise ValueError('原始 home 必须包含 22 个关节值')
    home = np.clip(home, lower, upper)
    base_torso = _forward_kinematics(root, joint_names, home)['torso_link'].copy()
    base_z = base_torso[2, 3]

    q = home.copy()
    sign = 1.0 if offset_mm >= 0 else -1.0
    done_mm = 0.0
    while True:
        done_mm = sign * min(abs(done_mm) + step_mm, abs(offset_mm))
        torso_target = base_torso.copy()
        torso_target[2, 3] = base_z + done_mm / 1000.0
        q, max_residual = _solve_torso_to_target(
            root, joint_names, lower, upper, q, torso_target)
        if max_residual > 2e-4:
            raise ValueError(
                f'躯干 IK 在 {done_mm:.0f} mm 处未收敛，最大躯干误差 {max_residual:.6f}；'
                '已接近该方向可达极限')
        if abs(done_mm - offset_mm) < 1e-9:
            break
    if not np.array_equal(q[8:], home[8:]):
        raise ValueError('躯干高度 IK 不得修改双臂关节')
    return q, base_z, _forward_kinematics(root, joint_names, q)['torso_link'][2, 3]


def _load_home(path):
    with open(path, encoding='utf-8') as stream:
        data = json.load(stream)
    qpos = data.get('qpos')
    if not isinstance(qpos, list) or len(qpos) != 22:
        raise ValueError('原始 home 文件必须包含 22 个 qpos')
    return [float(value) for value in qpos]


def _command_positions(q, feedback_head):
    """将躯干、双臂和头部目标转换成固定长度消息数组。

    ``feedback_head`` 参数保留用于兼容离线调用；实际动作传入的是 home
    目标中的两个头部关节角。头部不参与到位判定，但会随 home 命令移动。
    """
    values = np.asarray(q, dtype=float)
    if values.shape != (22,) or not np.all(np.isfinite(values)):
        raise ValueError('关节命令必须包含 22 个有限数值')
    head = np.asarray(feedback_head, dtype=float)
    if head.shape != (2,) or not np.all(np.isfinite(head)):
        raise ValueError('头部反馈必须包含 2 个有限数值')
    return (values[8:15].tolist(), values[15:22].tolist(),
            values[:6].tolist(), head.tolist() + [0.0])


def _format_joint_values(values):
    """格式化关节角；None 表示对应诊断 topic 没有数据。"""
    if values is None:
        return '<无数据>'
    return '[' + ', '.join(f'{float(value):+.5f}' for value in values) + ']'


def _format_joint_error(target, current):
    if target is None or current is None:
        return '<无数据>'
    target = np.asarray(target, dtype=float)
    current = np.asarray(current, dtype=float)
    if target.shape != current.shape:
        return '<长度不一致>'
    return _format_joint_values(current - target)


def _topic(namespace, suffix):
    namespace = '/' + namespace.strip('/') if namespace.strip('/') else ''
    return f'{namespace}/{suffix}'.replace('//', '/')


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offset-mm', type=float, required=True, help='相对原始 home 的高度偏移：负数下降，正数上升')
    parser.add_argument('--namespace', default=os.environ.get('APEX_ROS_NAMESPACE', 'tj'))
    parser.add_argument('--home-qpos', default=DEFAULT_HOME)
    parser.add_argument('--model-xml', default=DEFAULT_MODEL)
    parser.add_argument('--move-timeout-s', type=float, default=30.0)
    parser.add_argument('--command-hz', type=float, default=100.0)
    parser.add_argument('--source-index', type=int, default=2, help='mux user 源索引，默认 2')
    parser.add_argument('--max-offset-mm', type=float, default=DEFAULT_MAX_OFFSET_MM,
                        help='可选的人为偏移上限(mm)；默认 inf 不限制，幅度由 IK 和关节限位约束')
    parser.add_argument('--position-tolerance-rad', type=float, default=0.04)
    parser.add_argument('--joint-log-hz', type=float, default=1.0,
                        help='运行时关节诊断输出频率，单位 Hz，默认 1')
    parser.add_argument('--ramp-speed-rad-s', type=float, default=0.3,
                        help='起步平滑加减速阶段的单关节最大角速度，单位 rad/s，默认 0.3')
    parser.add_argument('--min-ramp-s', type=float, default=1.0,
                        help='即使位移很小也至少用这么多秒做平滑加减速，默认 1.0')
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    if not math.isfinite(args.offset_mm) or args.offset_mm == 0.0:
        raise ValueError('--offset-mm 必须是非零有限数值（单位 mm）')
    if args.max_offset_mm <= 0:
        raise ValueError('--max-offset-mm 必须为正')
    # 默认 max-offset-mm 为 inf，不设人为上限；仅当显式给出有限值时才按其裁剪。
    if math.isfinite(args.max_offset_mm) and abs(args.offset_mm) > args.max_offset_mm:
        raise ValueError(f'--offset-mm 超过人为上限 {args.max_offset_mm:g} mm')
    if not 10.0 <= args.command_hz <= 200.0:
        raise ValueError('--command-hz 必须在 10..200 Hz 内')
    if args.source_index < 0:
        raise ValueError('--source-index 必须非负')
    if not math.isfinite(args.move_timeout_s) or args.move_timeout_s <= 0:
        raise ValueError('--move-timeout-s 必须是正的有限秒数')
    if (not math.isfinite(args.position_tolerance_rad)
            or not 0 < args.position_tolerance_rad <= 0.2):
        raise ValueError('--position-tolerance-rad 必须在 0..0.2 rad 内')
    if (not math.isfinite(args.joint_log_hz) or not 0.1 <= args.joint_log_hz <= 10.0):
        raise ValueError('--joint-log-hz 必须在 0.1..10 Hz 内')
    if not math.isfinite(args.ramp_speed_rad_s) or not 0 < args.ramp_speed_rad_s <= 1.0:
        raise ValueError('--ramp-speed-rad-s 必须在 0..1 rad/s 内')
    if not math.isfinite(args.min_ramp_s) or not 0.2 <= args.min_ramp_s <= 10.0:
        raise ValueError('--min-ramp-s 必须在 0.2..10 s 内')
    if not os.path.isfile(args.model_xml) or not os.path.isfile(args.home_qpos):
        raise ValueError('MJCF 或原始 home 文件不存在')

    import rclpy
    from marvin_msgs.msg import JointcmdArm, JointcmdBody, JointcmdHead, Jointfeedback
    from marvin_msgs.srv import Int
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    home_q = _load_home(args.home_qpos)
    target_q, home_z, target_z = solve_height_offset(
        args.model_xml, home_q, args.offset_mm)
    rclpy.init()
    node = rclpy.create_node('torso_home_height_tool')
    feedback = {'q': None, 'received_at': None, 'active_source': None}
    # 记录三层命令链：本脚本 user 输入、joint_cmd_mux 输出、SDK 读回命令。
    command_state = {key: None for key in (
        'mux_A', 'mux_B', 'mux_body', 'sdk_A', 'sdk_B', 'sdk_body')}
    last_user_command = {'q': None}

    def feedback_callback(message):
        if len(message.arm_positions) >= 14 and len(message.body_positions) >= 6:
            head = list(message.head_positions[:2]) if len(message.head_positions) >= 2 else [0.0, 0.0]
            q = list(message.body_positions[:6]) + head + list(message.arm_positions[:14])
            if np.all(np.isfinite(q)):
                feedback['q'] = q
                feedback['received_at'] = time.monotonic()

    feedback_topic = _topic(args.namespace, 'info/joint_feedback')
    feedback_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Jointfeedback, feedback_topic, feedback_callback, feedback_qos)
    source_topic = _topic(args.namespace, 'info/joint_cmd_mux/active_source')
    source_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    from std_msgs.msg import Int32
    node.create_subscription(
        Int32, source_topic,
        lambda message: feedback.update(active_source=message.data), source_qos)

    command_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

    def command_callback(key, message):
        values = np.asarray(list(message.positions), dtype=float)
        if values.size and np.all(np.isfinite(values)):
            command_state[key] = values

    # mux 输出；这是机器人控制节点实际接收前的最终关节命令。
    for key, suffix, message_type in (
            ('mux_A', 'control/joint_cmd_A', JointcmdArm),
            ('mux_B', 'control/joint_cmd_B', JointcmdArm),
            ('mux_body', 'control/joint_cmd_body', JointcmdBody)):
        node.create_subscription(
            message_type, _topic(args.namespace, suffix),
            lambda message, key=key: command_callback(key, message), command_qos)
    # SDK 命令 topic 只有 publish_cmd_pos_debug=true 时才会发布；没有数据本身也是诊断结果。
    for key, suffix, message_type in (
            ('sdk_A', 'info/cmd_pos_A', JointcmdArm),
            ('sdk_B', 'info/cmd_pos_B', JointcmdArm),
            ('sdk_body', 'info/cmd_pos_body', JointcmdBody)):
        node.create_subscription(
            message_type, _topic(args.namespace, suffix),
            lambda message, key=key: command_callback(key, message), command_qos)

    select_name = _topic(args.namespace, 'control/joint_cmd_mux/select')
    select = node.create_client(Int, select_name)
    pubs = [
        node.create_publisher(JointcmdArm, _topic(args.namespace, 'control/user/joint_cmd_A'), 1),
        node.create_publisher(JointcmdArm, _topic(args.namespace, 'control/user/joint_cmd_B'), 1),
        node.create_publisher(JointcmdBody, _topic(args.namespace, 'control/user/joint_cmd_body'), 1),
        node.create_publisher(JointcmdHead, _topic(args.namespace, 'control/user/joint_cmd_head'), 1),
    ]

    def wait_service(client, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            if client.wait_for_service(timeout_sec=0.2):
                return True
            rclpy.spin_once(node, timeout_sec=0.0)
        return False

    def call_select(source):
        request = Int.Request(); request.data = source
        future = select.call_async(request)
        deadline = time.monotonic() + args.move_timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done():
            raise RuntimeError(f'切换 mux 源超时：{select_name}')
        response = future.result()
        if not response.success:
            raise RuntimeError(f'切换 mux 源失败：{response.message}')

    def publish(q):
        stamp = node.get_clock().now().to_msg()
        # mux 要求头部消息存在；动作目标使用 home 文件中的头部角度。
        left_q, right_q, body_q, head_q = _command_positions(q, q[6:8])
        left = JointcmdArm(); left.header.stamp = stamp; left.positions = left_q
        right = JointcmdArm(); right.header.stamp = stamp; right.positions = right_q
        body = JointcmdBody(); body.header.stamp = stamp; body.positions = body_q
        head = JointcmdHead(); head.header.stamp = stamp; head.positions = head_q
        for publisher, message in zip(pubs, (left, right, body, head)):
            publisher.publish(message)
        last_user_command['q'] = np.asarray(q, dtype=float).copy()

    def report_joint_state(label, target, current):
        """输出 user、mux、SDK 和反馈四层的躯干/双臂关节角（单位 rad）。"""
        target = np.asarray(target, dtype=float)
        current = np.asarray(current, dtype=float)
        user = last_user_command['q'] if last_user_command['q'] is not None else target
        node.get_logger().info(
            f'{label} 关节诊断(rad)：mux_source={feedback["active_source"]!r}；头部不参与')
        node.get_logger().info(
            '  user输入 body=' + _format_joint_values(user[:6]) +
            ' armL=' + _format_joint_values(user[8:15]) +
            ' armR=' + _format_joint_values(user[15:22]))
        node.get_logger().info(
            '  mux输出 body=' + _format_joint_values(command_state['mux_body']) +
            ' armL=' + _format_joint_values(command_state['mux_A']) +
            ' armR=' + _format_joint_values(command_state['mux_B']))
        node.get_logger().info(
            '  SDK命令 body=' + _format_joint_values(command_state['sdk_body']) +
            ' armL=' + _format_joint_values(command_state['sdk_A']) +
            ' armR=' + _format_joint_values(command_state['sdk_B']))
        node.get_logger().info(
            '  实时反馈 body=' + _format_joint_values(current[:6]) +
            ' armL=' + _format_joint_values(current[8:15]) +
            ' armR=' + _format_joint_values(current[15:22]))
        node.get_logger().info(
            '  反馈误差 body=' + _format_joint_error(target[:6], current[:6]) +
            ' armL=' + _format_joint_error(target[8:15], current[8:15]) +
            ' armR=' + _format_joint_error(target[15:22], current[15:22]))

    def require_fresh_feedback():
        received_at = feedback['received_at']
        if received_at is None or time.monotonic() - received_at > FEEDBACK_MAX_AGE_S:
            raise RuntimeError(f'关节反馈 {feedback_topic} 超过 {FEEDBACK_MAX_AGE_S:g}s 未更新')

    def require_active_source():
        if feedback['active_source'] != args.source_index:
            raise RuntimeError(
                f'mux 当前源为 {feedback["active_source"]!r}，期望 user 源 {args.source_index}；'
                '动作已停止，避免被其他控制器覆盖')

    def wait_feedback(target, label):
        deadline = time.monotonic() + args.move_timeout_s
        period = 1.0 / args.command_hz
        reached_since = None
        last_sample_at = None
        last_report_at = 0.0
        last_joint_log_at = 0.0
        joint_log_period = 1.0 / args.joint_log_hz
        while time.monotonic() < deadline and rclpy.ok():
            require_fresh_feedback()
            require_active_source()
            current = feedback['q']
            sample_at = feedback['received_at']
            if sample_at != last_sample_at:
                last_sample_at = sample_at
                error = np.abs(np.asarray(current)[CONTROLLED_INDICES] - target[CONTROLLED_INDICES])
                if np.max(error) <= args.position_tolerance_rad:
                    if reached_since is None:
                        reached_since = sample_at
                    elif sample_at - reached_since >= 0.3:
                        return
                else:
                    reached_since = None
                if sample_at - last_report_at >= 2.0:
                    worst_index = CONTROLLED_INDICES[int(np.argmax(error))]
                    node.get_logger().info(
                        f'{label}：最大关节误差 {np.max(error):.3f} rad，'
                        f'关节 {worst_index + 1}；头部不参与判定')
                    last_report_at = sample_at
            publish(target)
            rclpy.spin_once(node, timeout_sec=period)
            now = time.monotonic()
            if now - last_joint_log_at >= joint_log_period:
                report_joint_state(label, target, feedback['q'])
                last_joint_log_at = now
        raise RuntimeError(f'{label} 未在 {args.move_timeout_s:.1f}s 内到位')

    def ramp_then_hold(target, label):
        """先从当前反馈位置限速平滑过渡到目标，再按原逻辑保持到位。

        起步猛动的根因是第一帧 user 命令直接等于最终目标：mux 只在切源瞬间
        平滑，而 user 已是当前源（上次运行遗留）时不再过渡，机器人便从当前
        姿态一步跳向目标。这里在脚本内生成首尾速度为 0 的 smoothstep 轨迹，
        并按单关节最大角速度限制时长，使命令本身平滑，与 mux 是否过渡无关。
        """
        require_fresh_feedback()
        require_active_source()
        start = np.asarray(feedback['q'], dtype=float).copy()
        target = np.asarray(target, dtype=float)
        delta = target[CONTROLLED_INDICES] - start[CONTROLLED_INDICES]
        max_delta = float(np.max(np.abs(delta))) if delta.size else 0.0
        # smoothstep 峰值角速度是均值的 pi/2 倍，按此换算让 ramp-speed 成为真正的速度上限。
        ramp_s = max(args.min_ramp_s,
                     (math.pi / 2.0) * max_delta / args.ramp_speed_rad_s)
        period = 1.0 / args.command_hz
        node.get_logger().info(
            f'{label}：限速平滑起步，最大关节位移 {max_delta:.3f} rad，'
            f'限速 {args.ramp_speed_rad_s:.2f} rad/s，加减速 {ramp_s:.2f}s')
        # 必须按真实墙钟时间推进 phase：spin_once 一有回调就提前返回，
        # 若按循环步数推进会远快于 ramp_s，使限速失效。
        start_t = time.monotonic()
        next_pub = start_t
        while rclpy.ok():
            require_fresh_feedback()
            require_active_source()
            now = time.monotonic()
            if now >= next_pub:
                elapsed = now - start_t
                if elapsed >= ramp_s:
                    publish(target)
                    break
                # smoothstep：首尾速度为 0，避免起步和到点两端的突跳。
                phase = 0.5 - 0.5 * math.cos(math.pi * elapsed / ramp_s)
                publish(start + phase * (target - start))
                next_pub = now + period
            rclpy.spin_once(node, timeout_sec=period)
        wait_feedback(target, label)

    try:
        if not wait_service(select, args.move_timeout_s):
            raise RuntimeError(f'服务不可用：{select_name}')
        # 切源前必须确认能收到实时反馈，不能在失去反馈时盲目下发动作。
        feedback_deadline = time.monotonic() + min(5.0, args.move_timeout_s)
        while feedback['received_at'] is None and time.monotonic() < feedback_deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        require_fresh_feedback()
        # mux 在收到完整命令前会等待，并在切源时平滑过渡；切源前不发命令，
        # 避免 user 已是当前源时绕过 mux 过渡直接移动。
        call_select(args.source_index)
        source_deadline = time.monotonic() + 2.0
        while feedback['active_source'] != args.source_index and time.monotonic() < source_deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
        require_active_source()
        ramp_then_hold(np.asarray(home_q), '原始双手水平 home')
        node.get_logger().warning(
            f'原始 home 已到位；torso_link z {home_z:.3f} -> {target_z:.3f} m，'
            f'offset={args.offset_mm:.1f} mm；只调整 6 个躯干关节，双臂保持 home；'
            '放开躯干 X 前后位置约束，保持 Y/Z 和姿态')
        ramp_then_hold(target_q, '躯干高度目标')
        node.get_logger().info('躯干高度调整完成；user 源保持目标姿态')
        return 0
    except (OSError, ValueError, RuntimeError) as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
