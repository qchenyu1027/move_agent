"""提供给 LLM 的机器人原子动作。底层运动参数不暴露给模型。"""

import math

DEFAULT_GRASP_TAG_ID = 5
GRASP_OFFSET_CONFIGS = {
    3: "tag3_left_tool_offset.yaml",
    5: "tag5_left_tool_offset.yaml",
}


def _normalize_id(value, allowed, error_message):
    """兼容白名单内的整数字符串，不接受布尔值、小数或其他数字写法。"""
    if isinstance(value, str):
        value = value.strip()
        if value in {str(identifier) for identifier in allowed}:
            value = int(value)
    if type(value) is not int or value not in allowed:
        raise ValueError(error_message)
    return value


def move(robot, arguments):
    """移动到预设位置。用户说“移动到2”时调用并传入 pose_id=2。"""
    if not isinstance(arguments, dict) or set(arguments) != {"pose_id"}:
        raise ValueError("move_robot 只接受 pose_id")
    pose_id = _normalize_id(arguments["pose_id"], {1, 2, 3}, "pose_id 只能是 1、2、3")
    return robot.move_pose_ids([pose_id])


def grasp(robot, arguments):
    """抓取已标定物体。未指定标签时抓取默认目标 AprilTag 5。"""
    if not isinstance(arguments, dict) or not set(arguments).issubset({"tag_id"}):
        raise ValueError("grasp_object 只接受可选的 tag_id")
    tag_id = _normalize_id(
        arguments.get("tag_id", DEFAULT_GRASP_TAG_ID),
        GRASP_OFFSET_CONFIGS,
        f"tag_id 只能是已标定标签：{sorted(GRASP_OFFSET_CONFIGS)}",
    )
    return robot.grasp_apriltag(tag_id, GRASP_OFFSET_CONFIGS[tag_id])


def move_eef_delta(robot, arguments):
    """将选定 EEF 沿 base_link 坐标系移动一个不超过 200 mm 的短位移。"""
    if not isinstance(arguments, dict) or set(arguments) != {
        "side", "dx_mm", "dy_mm", "dz_mm",
    }:
        raise ValueError("move_eef_delta 需要 side、dx_mm、dy_mm、dz_mm")
    side = arguments["side"]
    if side not in {"left", "right"}:
        raise ValueError("side 只能是 left 或 right")
    values = tuple(arguments[name] for name in ("dx_mm", "dy_mm", "dz_mm"))
    if any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        for value in values
    ):
        raise ValueError("EEF 位移必须是有限数值")
    if not any(values):
        raise ValueError("EEF 位移不能全为 0")
    if math.dist(values, (0.0, 0.0, 0.0)) > 200.0:
        raise ValueError("EEF 单次位移不能超过 200 mm")
    return robot.move_eef_delta(side, *values)


MOVE_TOOL = {
    "type": "function",
    "function": {
        "name": "move_robot",
        "description": move.__doc__,
        "parameters": {
            "type": "object",
            "properties": {"pose_id": {"type": "integer", "enum": [1, 2, 3]}},
            "required": ["pose_id"],
            "additionalProperties": False,
        },
    },
}

GRASP_TOOL = {
    "type": "function",
    "function": {
        "name": "grasp_object",
        "description": grasp.__doc__,
        "parameters": {
            "type": "object",
            "properties": {
                "tag_id": {
                    "type": "integer",
                    "enum": [3, 5],
                    "description": "省略时使用默认目标 AprilTag 5",
                },
            },
            "additionalProperties": False,
        },
    },
}

EEF_DELTA_TOOL = {
    "type": "function",
    "function": {
        "name": "move_eef_delta",
        "description": move_eef_delta.__doc__,
        "parameters": {
            "type": "object",
            "properties": {
                "side": {"type": "string", "enum": ["left", "right"]},
                "dx_mm": {"type": "number"},
                "dy_mm": {"type": "number"},
                "dz_mm": {"type": "number"},
            },
            "required": ["side", "dx_mm", "dy_mm", "dz_mm"],
            "additionalProperties": False,
        },
    },
}

ROBOT_TOOL_SCHEMAS = [MOVE_TOOL, GRASP_TOOL, EEF_DELTA_TOOL]
ROBOT_TOOL_HANDLERS = {
    "move_robot": move,
    "grasp_object": grasp,
    "move_eef_delta": move_eef_delta,
}
