#!/usr/bin/python3
# 用途：读取固定站点位姿，并把世界位姿转换为 Apex MoveRelative 所需的机体位移；运行：python3 -m unittest -v agent_tools.test_native_nav_targets。
"""固定站点和世界位姿到机体相对动作的纯 Python 工具。"""

from __future__ import annotations

import ast
import math
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover - ROS 运行环境通常提供 PyYAML
    yaml = None


# 与项目根目录 pos.yaml 保持同一份站点数据；工具脚本独立运行时读取本地副本。
_POS_FILE = Path(__file__).with_name("pos.yaml")
_FALLBACK_POSES = (
    {"position": {"x": 1.1593247946463563, "y": -0.0606813495159879, "z": 0.0006836203517147192},
     "orientation": {"x": -0.003125499995759508, "y": 0.00028895241985152706, "z": 0.1580279854321458, "w": 0.9874296448742731}},
    {"position": {"x": 0.4683219141880431, "y": -0.502782612905945, "z": 1.739557635214539e-05},
     "orientation": {"x": -0.0061587499607713165, "y": 0.0059912608403982015, "z": -0.6770524077867871, "w": 0.7358846456494232}},
    {"position": {"x": -0.47073965883987257, "y": -0.3286442175793726, "z": 0.003886966394637969},
     "orientation": {"x": 0.0077290502851158115, "y": -0.01499700625787178, "z": 0.9998562617020962, "w": 0.0016755656062608399}},
    {"position": {"x": -0.4597338938554738, "y": -0.057127684192941855, "z": 0.016258406584268782},
     "orientation": {"x": 0.007439392623203332, "y": -0.011917645519320943, "z": 0.9993422215858315, "w": -0.03343275816573484}},
    {"position": {"x": 0.6257749412344397, "y": -0.6024240913864902, "z": -0.003826520948509977},
     "orientation": {"x": -0.0056950760914480725, "y": 0.005760373048980634, "z": -0.7054405978422937, "w": 0.708722757590411}},
)


def _load_poses():
    if yaml is None or not _POS_FILE.is_file():
        return _FALLBACK_POSES
    try:
        data = yaml.safe_load(_POS_FILE.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError):
        return _FALLBACK_POSES
    if not isinstance(data, dict):
        return _FALLBACK_POSES
    poses = []
    for index in range(1, 100):
        item = data.get(f"pose{index}")
        if item is None:
            break
        poses.append(item)
    return tuple(poses) or _FALLBACK_POSES


DEFAULT_POSES = _load_poses()


def parse_move_list(raw_value):
    """安全解析固定站点编号列表。"""
    try:
        value = ast.literal_eval(raw_value)
    except (SyntaxError, ValueError) as error:
        raise ValueError("move list 必须是形如 [1, 2, 3] 的整数列表") from error
    if not isinstance(value, (list, tuple)) or not value:
        raise ValueError("move list 必须是非空列表")
    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, int):
            raise ValueError(f"move list 只能包含整数编号 1..{len(DEFAULT_POSES)}")
        if not 1 <= item <= len(DEFAULT_POSES):
            raise ValueError(f"位姿编号 {item} 无效，编号范围为 1..{len(DEFAULT_POSES)}")
        result.append(item)
    return tuple(result)


def wrapped_angle(angle):
    """把角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_to_target(pose, index):
    """从 position/orientation 位姿计算 x、y、z、yaw。"""
    data = pose
    while isinstance(data, dict) and "position" not in data and "pose" in data:
        data = data["pose"]
    if not isinstance(data, dict):
        raise ValueError(f"默认位姿 {index} 结构无效")
    position, orientation = data.get("position"), data.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise ValueError(f"默认位姿 {index} 必须包含 position 和 orientation")
    try:
        values = tuple(float(position[key]) for key in ("x", "y", "z")) + tuple(
            float(orientation[key]) for key in ("x", "y", "z", "w"))
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"默认位姿 {index} 字段无效") from error
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"默认位姿 {index} 包含非有限数值")
    x, y, z, qx, qy, qz, qw = values
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        raise ValueError(f"默认位姿 {index} 的四元数模长无效")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return x, y, z, yaw


def quaternion_values_to_target(values):
    """把七维 ROS 位姿转换为 xyz/yaw。"""
    if len(values) != 7:
        raise ValueError("位姿需要 7 个数值：x y z qx qy qz qw")
    return pose_to_target({
        "position": dict(zip(("x", "y", "z"), values[:3])),
        "orientation": dict(zip(("x", "y", "z", "w"), values[3:])),
    }, "输入")


def unwrap_near(angle, reference):
    """返回与 reference 最近的 angle 等价角。"""
    return reference + wrapped_angle(angle - reference)


def world_to_body_delta(current, target):
    """把世界平移误差转换为当前 body 坐标系。"""
    x, y, _, yaw = current
    tx, ty, _, _ = target
    world_dx, world_dy = tx - x, ty - y
    return (
        math.cos(yaw) * world_dx + math.sin(yaw) * world_dy,
        -math.sin(yaw) * world_dx + math.cos(yaw) * world_dy,
    )


def relative_step(current, target, max_step_m=0.10, max_turn_rad=math.pi / 6):
    """兼容旧离线测试的限幅转换；新导航默认不调用此函数。"""
    values = (*current, *target, max_step_m, max_turn_rad)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("位姿和动作限制必须是有限数值")
    dx, dy = world_to_body_delta(current, target)
    distance = math.hypot(dx, dy)
    if distance > max_step_m:
        scale = max_step_m / distance
        dx, dy = dx * scale, dy * scale
    turn = max(-max_turn_rad, min(max_turn_rad, wrapped_angle(target[3] - current[3])))
    return dx, dy, turn, distance
