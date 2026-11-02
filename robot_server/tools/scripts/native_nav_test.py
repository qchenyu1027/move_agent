#!/usr/bin/env python3
# 用途：读取 VLAHost 实时世界位姿，转换为 body-frame 后通过 Apex MoveRelative 到固定站点；运行：source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh && /usr/bin/python3 tools/scripts/native_nav_test.py --pose-ids '[3,2,1]' --wait --execute。
"""固定站点闭环导航：VLAHost 位姿 + Apex ``/move/MoveRelativeCmd``。"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import ProxyHandler, Request, build_opener

try:  # 预览和 --help 不需要 ROS；真实执行时由 apex_ros_env 提供这些包。
    import rclpy
    from action_msgs.msg import GoalStatus
    from move.action import MoveRelative
    from move.msg import State as MoveState
    from move.srv import SetParkingMode
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
except ImportError:  # pragma: no cover - 仅覆盖未加载 ROS 环境的预览路径
    rclpy = None
    GoalStatus = MoveRelative = MoveState = SetParkingMode = ActionClient = None
    SignalHandlerOptions = None
    Node = object

from native_nav_targets import (DEFAULT_POSES, parse_move_list, pose_to_target,
                                quaternion_values_to_target, unwrap_near,
                                world_to_body_delta, wrapped_angle)

DEFAULT_VLAHOST_URL = os.environ.get("VLAHOST_URL", "http://6.6.7.100:8011").rstrip("/")
POSE_MAX_AGE_S = 0.5
DEFAULT_YAW_MAX_TRAVEL_RAD = math.pi + 0.35


class PoseNotReadyError(RuntimeError):
    """VLAHost 位姿无效或过期。"""


class NavigationError(RuntimeError):
    """导航失败，retryable 表示是否允许同一站点再尝试一次。"""

    def __init__(self, message, *, retryable=False):
        super().__init__(message)
        self.retryable = retryable


def request_json(base_url, path, body=None, timeout=3.0):
    """访问 VLAHost JSON 接口，不使用系统代理。"""
    if body is None:
        request = Request(base_url.rstrip("/") + path, method="GET")
    else:
        request = Request(
            base_url.rstrip("/") + path,
            data=json.dumps(body).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
    try:
        with build_opener(ProxyHandler({})).open(request, timeout=timeout) as response:
            result = json.load(response)
    except HTTPError as error:
        detail = error.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"VLAHost HTTP {error.code}: {detail}") from error
    except (URLError, OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"无法访问 VLAHost {base_url}: {error}") from error
    if not isinstance(result, dict):
        raise RuntimeError("VLAHost 响应不是 JSON 对象")
    return result


def validate_base_world_pose(pose):
    """校验 camera_init -> body 的新鲜世界位姿并返回 x/y/z/yaw。"""
    if not isinstance(pose, dict) or not pose.get("valid"):
        raise PoseNotReadyError("VLAHost 世界位姿无效")
    if pose.get("frame_id") != "camera_init" or pose.get("child_frame_id") != "body":
        raise RuntimeError(
            "世界位姿坐标系错误，需要 camera_init -> body，实际为 "
            f"{pose.get('frame_id')} -> {pose.get('child_frame_id')}"
        )
    try:
        age = float(pose["received_age_s"])
        stamp = float(pose["stamp_s"])
        position = pose["position"]
        x, y, z = (float(position[key]) for key in ("x", "y", "z"))
        yaw = float(pose["yaw_rad"])
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError("VLAHost 世界位姿字段无效") from error
    if not all(math.isfinite(value) for value in (age, stamp, x, y, z, yaw)):
        raise RuntimeError("VLAHost 世界位姿包含非有限数值")
    if age < 0.0 or age > POSE_MAX_AGE_S:
        raise PoseNotReadyError(f"VLAHost 世界位姿过期：{age:.3f} s")
    return x, y, z, yaw, stamp


def pose_dict_to_target(pose):
    """将 VLAHost 风格位姿字典转换为目标 xyz/yaw。"""
    try:
        position = pose["position"]
        return (
            float(position["x"]), float(position["y"]), float(position["z"]),
            float(pose["yaw_rad"]),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError("世界目标位姿字段无效") from error


class NativeNavTest(Node):
    """只负责 Apex Action；世界位姿全部从 VLAHost 读取。"""

    def __init__(self, args):
        super().__init__("rpent_native_nav_test")
        self.args = args
        self.relative_client = ActionClient(self, MoveRelative, "/move/MoveRelativeCmd")
        self.parking_client = self.create_client(SetParkingMode, "/move/SetParkingMode")
        self.last_move_state = None
        self.move_at = 0.0
        self.active_relative_goal = None
        self.relative_send_future = None
        self.cable_reference_yaw = args.yaw_reference_rad
        self.last_unwrapped_yaw = args.yaw_reference_rad
        self.create_subscription(MoveState, "/move/State", self._move_cb, 10)

    def _move_cb(self, msg):
        self.last_move_state = msg
        self.move_at = time.monotonic()

    def get_pose(self):
        response = request_json(self.args.vlahost_url, "/base_world_pose",
                                timeout=self.args.http_timeout_s)
        pose = response.get("pose")
        return pose, validate_base_world_pose(pose)

    def wait_for_fresh_pose(self, *, newer_than=None, stable_s=None):
        """要求位姿有效、未过期且源时间戳持续更新。"""
        stable_s = self.args.pose_stable_s if stable_s is None else stable_s
        deadline = time.monotonic() + self.args.pose_ready_timeout_s
        first_stamp = None
        stable_started = None
        last_stamp = None
        updates = 0
        while time.monotonic() < deadline:
            try:
                pose, values = self.get_pose()
            except PoseNotReadyError as error:
                time.sleep(0.1)
                last_error = error
                continue
            stamp = values[4]
            if newer_than is not None and stamp <= newer_than:
                time.sleep(0.05)
                last_error = PoseNotReadyError("等待动作后的新世界位姿")
                continue
            if first_stamp is None:
                first_stamp = stamp
                stable_started = time.monotonic()
                last_stamp = stamp
                updates = 1
            elif stamp < last_stamp:
                raise RuntimeError("世界位姿时间戳倒退，停止移动")
            elif stamp > last_stamp:
                last_stamp = stamp
                updates += 1
            if stable_s <= 0.0 or (stable_started is not None and
                                     time.monotonic() - stable_started >= stable_s and
                                     updates >= 2):
                return pose, values
            time.sleep(0.1)
        raise RuntimeError(
            f"等待 VLAHost 实时世界位姿超时：{locals().get('last_error', '无有效位姿')}"
        )

    def release_parking(self):
        """仅在用户显式指定时解除驻车。"""
        if not self.args.release_parking:
            return
        deadline = time.monotonic() + self.args.service_timeout
        while self.last_move_state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.last_move_state is not None and not self.last_move_state.is_parking_mode:
            return
        if not self.parking_client.wait_for_service(timeout_sec=self.args.service_timeout):
            raise NavigationError("/move/SetParkingMode 不可用")
        request = SetParkingMode.Request()
        request.enable = False
        future = self.parking_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.service_timeout)
        if not future.done() or future.result() is None or future.result().error_code != 0:
            raise NavigationError("解除底盘驻车失败")
        print("已请求解除底盘驻车。", flush=True)

    def relative_move(self, dx, dy, turn):
        """发送一个 Apex MoveRelative goal，不拆分、不调用 CreateTaskByPose。"""
        if not self.relative_client.wait_for_server(timeout_sec=self.args.service_timeout):
            raise NavigationError("/move/MoveRelativeCmd Action 不可用")
        goal = MoveRelative.Goal()
        goal.relative_pose.position.x = float(dx)
        goal.relative_pose.position.y = float(dy)
        # Apex 接口约定：orientation.w 直接承载相对 yaw（弧度）。
        goal.relative_pose.orientation.w = float(turn)
        send_future = self.relative_client.send_goal_async(goal)
        self.relative_send_future = send_future
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self.args.service_timeout)
        self.relative_send_future = None
        if not send_future.done() or send_future.result() is None:
            raise NavigationError("MoveRelative goal 发送超时", retryable=True)
        handle = send_future.result()
        if not handle.accepted:
            raise NavigationError("MoveRelative goal 被拒绝", retryable=True)
        self.active_relative_goal = handle
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.args.move_timeout_s)
        if not result_future.done() or result_future.result() is None:
            self._cancel_active_goal()
            raise NavigationError("MoveRelative 结果超时", retryable=True)
        self.active_relative_goal = None
        response = result_future.result()
        result = response.result
        print(
            f"MoveRelative status={response.status} error_code={result.error_code} "
            f"message={result.error_msg}", flush=True)
        if response.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != 0:
            raise NavigationError(
                f"MoveRelative 执行失败：error_code={result.error_code} message={result.error_msg}",
                retryable=False,
            )

    def _cancel_active_goal(self):
        if self.active_relative_goal is None:
            return
        future = self.active_relative_goal.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.service_timeout)
        self.active_relative_goal = None

    def _turn_for_target(self, current_yaw, target_yaw):
        """按首次导航的线缆参考方向选择等价目标角。"""
        if self.cable_reference_yaw is None:
            self.cable_reference_yaw = current_yaw
        if self.last_unwrapped_yaw is None:
            self.last_unwrapped_yaw = unwrap_near(current_yaw, self.cable_reference_yaw)
        current = unwrap_near(current_yaw, self.last_unwrapped_yaw)
        target = unwrap_near(target_yaw, self.cable_reference_yaw)
        turn = target - current
        if abs(turn) > self.args.yaw_max_travel_rad:
            raise NavigationError(
                f"目标 yaw 单次相对旋转 {math.degrees(turn):.1f}° 超过安全上限 "
                f"{math.degrees(self.args.yaw_max_travel_rad):.1f}°"
            )
        return turn, target

    def move_to_target_once(self, target):
        _, current = self.wait_for_fresh_pose()
        current_x, current_y, current_z, current_yaw, stamp = current
        target_x, target_y, target_z, target_yaw = target
        z_error = target_z - current_z
        if abs(z_error) > self.args.position_tolerance_m:
            raise NavigationError(
                f"底盘不能主动调整 z，当前与目标相差 {z_error:.3f} m"
            )
        distance = math.hypot(target_x - current_x, target_y - current_y)
        yaw_error = abs(wrapped_angle(target_yaw - current_yaw))
        if distance <= self.args.position_tolerance_m and yaw_error <= self.args.yaw_tolerance_rad:
            print(f"目标已在容差内：XY={distance:.4f} m yaw={yaw_error:.4f} rad")
            return
        dx, dy = world_to_body_delta(current[:4], target)
        turn, target_unwrapped = self._turn_for_target(current_yaw, target_yaw)
        print(
            f"发送 Apex MoveRelative：body_dx={dx:.4f} body_dy={dy:.4f} "
            f"yaw={math.degrees(turn):.2f}°（世界误差 {distance:.4f} m）",
            flush=True,
        )
        self.relative_move(dx, dy, turn)
        self.last_unwrapped_yaw = target_unwrapped
        _, final = self.wait_for_fresh_pose(newer_than=stamp, stable_s=0.0)
        fx, fy, fz, fyaw, _ = final
        final_xy = math.hypot(fx - target_x, fy - target_y)
        final_z = abs(fz - target_z)
        final_yaw = abs(wrapped_angle(fyaw - target_yaw))
        print(
            f"到站复核：XY误差={final_xy:.4f} m Z误差={final_z:.4f} m "
            f"yaw误差={final_yaw:.4f} rad", flush=True,
        )
        if (final_xy > self.args.position_tolerance_m or
                final_z > self.args.position_tolerance_m or
                final_yaw > self.args.yaw_tolerance_rad):
            raise NavigationError("MoveRelative 返回成功但到站误差超限", retryable=True)

    def move_to_target(self, target):
        """每个固定站点最多两次；只有明确可恢复失败才重试。"""
        self.release_parking()
        for attempt in range(1, self.args.max_attempts + 1):
            try:
                self.move_to_target_once(target)
                return 0
            except NavigationError as error:
                print(
                    f"导航尝试 {attempt}/{self.args.max_attempts} 失败：{error} "
                    f"retryable={str(error.retryable).lower()}",
                    file=sys.stderr,
                )
                if not error.retryable or attempt >= self.args.max_attempts:
                    return 5
                print("失败标记为 retryable，重新读取实时位姿后重试。", flush=True)
        return 5

    def cancel_active(self):
        self._cancel_active_goal()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose", nargs=4, type=float, metavar=("X", "Y", "Z", "YAW"))
    parser.add_argument("--pose-quat", nargs=7, type=float,
                        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    parser.add_argument("--pose-ids", help='固定站点编号列表，例如 "[3,2,1]"')
    parser.add_argument("--fixed", action="store_true", help="使用固定站点 1")
    parser.add_argument("--offset-x-m", type=float)
    parser.add_argument("--offset-y-m", type=float)
    parser.add_argument("--vlahost-url", default=DEFAULT_VLAHOST_URL)
    parser.add_argument("--location-topic", help=argparse.SUPPRESS)  # 兼容旧任务，已不使用
    parser.add_argument("--relative", nargs=3, type=float, metavar=("DX", "DY", "YAW_DEG"),
                        help="直接发送一个 body-frame 相对动作")
    parser.add_argument("--speed-m-s", "--max-speed", type=float, help="兼容旧参数；MoveRelative 不支持速度")
    parser.add_argument("--release-parking", action="store_true")
    parser.add_argument("--service-timeout", type=float, default=5.0)
    parser.add_argument("--http-timeout-s", type=float, default=3.0)
    parser.add_argument("--pose-ready-timeout-s", type=float, default=10.0)
    parser.add_argument("--pose-stable-s", type=float, default=0.5)
    parser.add_argument("--move-timeout-s", "--wait-timeout", type=float, default=120.0)
    parser.add_argument("--position-tolerance-m", "--arrival-tolerance", type=float, default=0.05)
    parser.add_argument("--yaw-tolerance-rad", type=float, default=0.10)
    parser.add_argument("--yaw-reference-rad", type=float)
    parser.add_argument("--yaw-max-travel-rad", type=float, default=DEFAULT_YAW_MAX_TRAVEL_RAD)
    parser.add_argument("--max-attempts", type=int, default=2)
    parser.add_argument("--pause-s", type=float, default=0.5)
    parser.add_argument("--wait", action="store_true", help="兼容旧命令；Action 始终等待结果")
    parser.add_argument("--execute", action="store_true", help="实际发送 MoveRelative goal")
    args = parser.parse_args()
    if (args.offset_x_m is None) != (args.offset_y_m is None):
        parser.error("--offset-x-m 和 --offset-y-m 必须同时指定")
    modes = [args.pose is not None, args.pose_quat is not None, args.pose_ids is not None,
             args.fixed, args.offset_x_m is not None, args.relative is not None]
    if sum(modes) != 1:
        parser.error("必须且只能指定一种目标：--pose、--pose-quat、--pose-ids、--fixed、世界偏移或 --relative")
    if args.pose_ids is not None:
        try:
            args.pose_ids = parse_move_list(args.pose_ids)
        except ValueError as error:
            parser.error(str(error))
    if args.pose_quat is not None:
        try:
            args.pose = quaternion_values_to_target(args.pose_quat)
        except ValueError as error:
            parser.error(str(error))
    if args.fixed:
        args.pose = pose_to_target(DEFAULT_POSES[0], 1)
    values = [args.service_timeout, args.http_timeout_s, args.pose_ready_timeout_s,
              args.pose_stable_s, args.move_timeout_s, args.position_tolerance_m,
              args.yaw_tolerance_rad, args.yaw_max_travel_rad, args.max_attempts,
              args.pause_s, *(args.pose or ()), *(args.relative or ())]
    if any(not math.isfinite(value) for value in values):
        parser.error("导航参数必须是有限数值")
    if args.pose_ready_timeout_s <= 0 or args.pose_stable_s < 0 or args.move_timeout_s <= 0:
        parser.error("超时时间无效")
    if args.position_tolerance_m <= 0 or args.yaw_tolerance_rad <= 0:
        parser.error("位置和 yaw 容差必须为正数")
    if args.max_attempts < 1 or args.max_attempts > 2:
        parser.error("--max-attempts 必须为 1 或 2")
    if args.pause_s < 0:
        parser.error("--pause-s 不能为负数")
    return args


def main():
    args = parse_args()
    if not args.execute:
        if args.pose_ids is not None:
            targets = [pose_to_target(DEFAULT_POSES[index - 1], index) for index in args.pose_ids]
        elif args.pose is not None:
            targets = [args.pose]
        else:
            targets = [args.relative]
        print(f"DRY-RUN Apex MoveRelative targets={targets} VLAHost={args.vlahost_url}")
        print("Pass --execute to send the real MoveRelative request.")
        return 0
    if rclpy is None:
        print("ERROR: 未加载 ROS 环境，请先 source apex_ros_env.sh。", file=sys.stderr)
        return 2
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = NativeNavTest(args)
    try:
        if args.speed_m_s is not None:
            print("注意：MoveRelative 不支持速度字段，--speed-m-s 参数不生效。", file=sys.stderr)
        if args.relative is not None:
            dx, dy, yaw_deg = args.relative
            node.release_parking()
            node.relative_move(dx, dy, math.radians(yaw_deg))
            return 0
        if args.offset_x_m is not None:
            _, current = node.wait_for_fresh_pose()
            x, y, z, yaw, _ = current
            targets = [(x + args.offset_x_m, y + args.offset_y_m, z, yaw)]
        elif args.pose_ids is not None:
            targets = [pose_to_target(DEFAULT_POSES[index - 1], index) for index in args.pose_ids]
        else:
            targets = [args.pose]
        for index, target in enumerate(targets, 1):
            print(f"导航目标 {index}/{len(targets)}：world xyz/yaw={target}", flush=True)
            result = node.move_to_target(target)
            if result != 0:
                return result
            if index < len(targets) and args.pause_s:
                time.sleep(args.pause_s)
        return 0
    except KeyboardInterrupt:
        print("收到 Ctrl-C，正在取消 MoveRelative。", file=sys.stderr)
        node.cancel_active()
        return 130
    except (NavigationError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        node.cancel_active()
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
