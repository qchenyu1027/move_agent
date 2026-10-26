"""提供给 LLM 的两个机器人动作。底层运动参数不暴露给模型。"""

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

ROBOT_TOOL_SCHEMAS = [MOVE_TOOL, GRASP_TOOL]
ROBOT_TOOL_HANDLERS = {"move_robot": move, "grasp_object": grasp}
