#!/usr/bin/env python3
# 用途：把实时水瓶位置和抓取航路发布为 RViz 标记，不发送任何机器人运动命令。
# 运行：source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh && /usr/bin/python3 -B -m path_planner.rviz_grasp_preview --table-z 0.70
"""在 base_link 中显示物体、工具和预抓取点闭合航路。"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

from .core import PlanError, build_grasp_plan

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'tools' / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from left_tool_accuracy_test import (multiply_quaternions, read_tool_mount,
                                     rotate_vector, transform_pose)


COLORS = {
    'current': (0.7, 0.7, 0.7),
    'lift_before_translation': (0.0, 0.8, 0.8),
    'above_pregrasp': (0.1, 0.4, 1.0),
    'pregrasp': (1.0, 0.85, 0.0),
    'grasp': (1.0, 0.1, 0.1),
    'lift_after_grasp': (0.1, 0.9, 0.2),
    'object': (0.9, 0.1, 0.8),
}
POINT_IDS = {name: index for index, name in enumerate(COLORS)}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right'), default='left')
    parser.add_argument('--table-z', type=float, required=True,
                        help='桌面在 base_link 中的 Z，单位 m')
    parser.add_argument('--object-pose-topic', default='/object_pose')
    parser.add_argument('--marker-topic', default='/f2g_pose/grasp_markers')
    parser.add_argument('--model-xml', help='与抓取规划器相同的 Gento_Luna.xml')
    parser.add_argument('--grasp-offset-mm', nargs=3, type=float, default=None,
                        metavar=('X', 'Y', 'Z'))
    parser.add_argument('--approach-front-mm', type=float, default=None)
    parser.add_argument('--approach-height-mm', type=float, default=None)
    parser.add_argument('--overhead-mm', type=float, default=None)
    parser.add_argument('--lift-height-mm', type=float, default=None)
    parser.add_argument('--table-clearance-mm', type=float, default=None)
    parser.add_argument('--start-clearance-mm', type=float, default=None)
    parser.add_argument('--display-seconds', type=float, default=30.0,
                        help='最后一帧有效物体位姿保留多久，默认 30 秒')
    args = parser.parse_args(argv)
    values = [args.table_z, args.display_seconds]
    for name in ('approach_front_mm', 'approach_height_mm', 'overhead_mm',
                 'lift_height_mm', 'table_clearance_mm', 'start_clearance_mm'):
        value = getattr(args, name)
        if value is not None:
            values.append(value)
            if value <= 0.0:
                parser.error(f'--{name.replace("_", "-")} 必须为正数')
    if args.grasp_offset_mm is not None:
        values.extend(args.grasp_offset_mm)
    if not all(math.isfinite(value) for value in values) or args.display_seconds <= 0.0:
        parser.error('高度、偏移和显示时间必须是有限数值，显示时间必须大于零')
    return args


def plan_options(args):
    names = ('grasp_offset_mm', 'approach_front_mm', 'approach_height_mm',
             'overhead_mm', 'lift_height_mm', 'table_clearance_mm',
             'start_clearance_mm')
    return {name: getattr(args, name) for name in names if getattr(args, name) is not None}


def _point(values):
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def make_markers(waypoints, object_position, stamp):
    """用固定 ID 标记关键点，返回标记和本帧使用的 ID。"""
    markers = []
    active_ids = set()

    def base_marker(identifier, marker_type):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = stamp
        marker.ns = 'f2g_grasp_preview'
        marker.id = identifier
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        active_ids.add(identifier)
        return marker

    for name, position in [('object', object_position)] + [
        (point.name, point.position) for point in waypoints
    ]:
        identifier = POINT_IDS[name]
        color = COLORS[name]
        sphere = base_marker(identifier, Marker.SPHERE)
        sphere.pose.position = _point(position)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.032 if name == 'pregrasp' else 0.022
        sphere.color.r, sphere.color.g, sphere.color.b = color
        markers.append(sphere)

        label = base_marker(identifier + 100, Marker.TEXT_VIEW_FACING)
        label.pose.position = _point((position[0], position[1], position[2] + 0.045))
        label.scale.z = 0.026
        label.color.r = label.color.g = label.color.b = 1.0
        label.text = name
        markers.append(label)

    path = base_marker(200, Marker.LINE_STRIP)
    path.scale.x = 0.006
    path.color.r, path.color.g, path.color.b = (1.0, 0.55, 0.0)
    path.points = [_point(point.position) for point in waypoints]
    markers.append(path)

    # 删除旧版本留下的红色抓取点标记，避免它被误认为仍需前进的目标。
    for identifier in (POINT_IDS['grasp'], POINT_IDS['grasp'] + 100):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = stamp
        marker.ns = 'f2g_grasp_preview'
        marker.id = identifier
        marker.action = Marker.DELETE
        markers.append(marker)

    named = {point.name: point for point in waypoints}
    if 'above_pregrasp' in named and 'pregrasp' in named:
        arrow = base_marker(201, Marker.ARROW)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = (0.008, 0.018, 0.025)
        arrow.color.r, arrow.color.g, arrow.color.b = (1.0, 0.1, 0.1)
        arrow.points = [_point(named['above_pregrasp'].position), _point(named['pregrasp'].position)]
        markers.append(arrow)
    return MarkerArray(markers=markers), active_ids


class GraspPreview(Node):
    def __init__(self, args):
        super().__init__('f2g_grasp_rviz_preview')
        self.args = args
        self.arm_link = 'Arm_L7_Link' if args.side == 'left' else 'Arm_R7_Link'
        model_xml = args.model_xml or str(Path(
            get_package_share_directory('marvin_description')) /
            'mjcf/Gento_Luna/Gento_Luna.xml')
        self.mount_position, self.mount_quaternion = read_tool_mount(
            model_xml, 'L' if args.side == 'left' else 'R')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, args.marker_topic, qos)
        self.create_subscription(PoseStamped, args.object_pose_topic,
                                 self.on_pose, qos_profile_sensor_data)
        self.pending_pose = None
        self.last_received = None
        self.last_stamp = None
        self.active_ids = set()
        self.last_warning = 0.0
        self.create_timer(0.2, self.tick)
        self.get_logger().info(
            f'等待 {args.object_pose_topic}；发布 {args.marker_topic}；只显示，不执行运动')

    def on_pose(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if stamp <= 0.0 or not -0.05 <= age <= 0.5 or stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        self.pending_pose = message
        self.last_received = time.monotonic()

    def object_in_base(self, message):
        frame = message.header.frame_id.strip('/')
        if not frame:
            raise ValueError('物体位姿 frame_id 为空')
        p = message.pose.position
        raw = (p.x, p.y, p.z)
        if frame == 'base_link':
            position = raw
        else:
            transform = self.buffer.lookup_transform(
                'base_link', frame, Time(), timeout=Duration(seconds=0.05))
            origin, rotation = transform_pose(transform)
            position = tuple(a + b for a, b in zip(origin, rotate_vector(raw, rotation)))
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

    def clear(self):
        if not self.active_ids:
            return
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        self.publisher.publish(MarkerArray(markers=[marker]))
        self.active_ids.clear()

    def tick(self):
        if self.last_received is None:
            return
        if time.monotonic() - self.last_received > self.args.display_seconds:
            self.pending_pose = None
            self.last_received = None
            self.clear()
            self.get_logger().warning('物体位姿已过期，清除 RViz 抓取标记')
            return
        if self.pending_pose is None:
            return
        try:
            object_position = self.object_in_base(self.pending_pose)
            tool_position, tool_quaternion = self.actual_tool()
            waypoints = build_grasp_plan(
                tool_position, tool_quaternion, object_position,
                table_z_m=self.args.table_z, **plan_options(self.args))
        except (PlanError, ValueError, TransformException) as error:
            if time.monotonic() - self.last_warning >= 2.0:
                self.get_logger().warning(f'等待有效抓取航路：{error}')
                self.last_warning = time.monotonic()
            self.clear()
            return
        stamps = self.get_clock().now().to_msg()
        array, active_ids = make_markers(waypoints, object_position, stamps)
        removed = self.active_ids - active_ids
        for identifier in removed:
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = stamps
            marker.ns = 'f2g_grasp_preview'
            marker.id = identifier
            marker.action = Marker.DELETE
            array.markers.append(marker)
        self.publisher.publish(array)
        self.active_ids = active_ids
        self.pending_pose = None
        pregrasp = next(point for point in waypoints if point.name == 'pregrasp')
        self.get_logger().info(
            f'夹爪闭合点 base_link xyz={pregrasp.position}', throttle_duration_sec=2.0)


def main(argv=None):
    args = parse_args(argv)
    rclpy.init(args=[])
    node = None
    try:
        node = GraspPreview(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.clear()
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
