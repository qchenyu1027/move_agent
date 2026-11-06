#!/usr/bin/env python3
# 用途：录制 RealSense RGB-D、TF、关节状态和 VLAHost 世界位姿；运行：source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh && /usr/bin/python3 tools/scripts/record_semantic_dataset.py --output semantic_bag。
"""录制语义地图离线数据，不发送任何底盘运动指令。"""

from __future__ import annotations

import argparse
import json
import math
import signal
import subprocess
import sys
import time
from urllib.request import ProxyHandler, Request, build_opener

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
except ImportError:  # pragma: no cover - 仅在未加载 ROS 环境时触发
    rclpy = None
    PoseStamped = None
    Node = object


DEFAULT_TOPICS = (
    "/camera/camera/color/image_raw",
    "/camera/camera/color/camera_info",
    "/camera/camera/depth/image_rect_raw",
    "/camera/camera/depth/camera_info",
    "/camera/camera/aligned_depth_to_color/image_raw",
    "/camera/camera/aligned_depth_to_color/camera_info",
    "/tf",
    "/tf_static",
    "/tj/joint_states",
    "/controller/odom",
    "/odomAftUpdate",
    "/slam/LocationData",
    "/semantic/base_world_pose",
)


def read_world_pose(url: str, timeout: float):
    request = Request(url.rstrip("/") + "/base_world_pose", method="GET")
    with build_opener(ProxyHandler({})).open(request, timeout=timeout) as response:
        payload = json.load(response)
    pose = payload.get("pose") if isinstance(payload, dict) else None
    if not isinstance(pose, dict) or not pose.get("valid"):
        return None
    position = pose.get("position", {})
    try:
        x = float(position["x"])
        y = float(position["y"])
        z = float(position["z"])
        yaw = float(pose["yaw_rad"])
        age = float(pose["received_age_s"])
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (x, y, z, yaw, age)):
        return None
    if age < 0.0 or age > 0.5:
        return None
    return x, y, z, yaw


class WorldPosePublisher(Node):
    def __init__(self, url: str, timeout: float):
        super().__init__("semantic_world_pose_recorder")
        self.url = url
        self.timeout = timeout
        self.publisher = self.create_publisher(PoseStamped, "/semantic/base_world_pose", 10)

    def publish_once(self):
        try:
            values = read_world_pose(self.url, self.timeout)
        except Exception as error:
            self.get_logger().warning(f"VLAHost 位姿读取失败：{error}")
            return False
        if values is None:
            return False
        x, y, z, yaw = values
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "camera_init"
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)
        self.publisher.publish(message)
        return True


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="rosbag2 输出目录")
    parser.add_argument("--vlahost-url", default="http://6.6.7.100:8011")
    parser.add_argument("--http-timeout-s", type=float, default=0.5)
    parser.add_argument("--pose-rate", type=float, default=10.0)
    parser.add_argument("--duration-s", type=float, help="录制时长；省略则按 Ctrl-C 结束")
    parser.add_argument("--topics", nargs="*", default=list(DEFAULT_TOPICS),
                        help="覆盖默认 ROS 话题列表")
    return parser.parse_args()


def main():
    args = parse_args()
    if rclpy is None:
        print("ERROR: 未加载 ROS 环境，请先 source apex_ros_env.sh。", file=sys.stderr)
        return 2
    if args.pose_rate <= 0 or args.http_timeout_s <= 0:
        raise SystemExit("--pose-rate 和 --http-timeout-s 必须为正数")
    rclpy.init(args=[])
    node = WorldPosePublisher(args.vlahost_url, args.http_timeout_s)
    # ROS 2 Humble 的 bag record 将话题作为位置参数，不支持 --topics。
    command = ["ros2", "bag", "record", "-o", args.output, *args.topics]
    print("开始录制：" + " ".join(command), flush=True)
    bag_process = subprocess.Popen(command)
    start = time.monotonic()
    period = 1.0 / args.pose_rate
    next_pose = start
    try:
        while rclpy.ok() and bag_process.poll() is None:
            now = time.monotonic()
            if now >= next_pose:
                node.publish_once()
                next_pose = now + period
            rclpy.spin_once(node, timeout_sec=0.05)
            if args.duration_s is not None and now - start >= args.duration_s:
                break
    except KeyboardInterrupt:
        print("收到 Ctrl-C，停止录制。", file=sys.stderr)
    finally:
        if bag_process.poll() is None:
            bag_process.send_signal(signal.SIGINT)
            try:
                bag_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                bag_process.terminate()
        node.destroy_node()
        rclpy.shutdown()
    return bag_process.returncode or 0


if __name__ == "__main__":
    raise SystemExit(main())
