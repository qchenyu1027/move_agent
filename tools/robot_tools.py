"""真实机器人模型工具及严格参数校验。"""

MOVE_OPTIONS = {"vlahost-url", "speed-m-s", "yaw-speed-rad-s", "yaw-command-hz",
                "yaw-command-duration-s", "position-tolerance-m", "yaw-tolerance-rad",
                "move-timeout-s", "yaw-timeout-s", "http-timeout-s", "pose-ready-timeout-s",
                "pose-stable-s", "odometry-retries", "pause-s"}


def _options(value, allowed=MOVE_OPTIONS):
    if value is None:
        return None
    if not isinstance(value, dict) or not set(value).issubset(allowed):
        raise ValueError(f"options 只能包含：{sorted(allowed)}")
    return value


def health(robot, arguments):
    """只读检查机器人健康状态、连接、忙闲和运动准入条件。执行任何动作前必须调用。"""
    if not isinstance(arguments, dict) or arguments:
        raise ValueError("robot_health 不接受参数")
    return robot.health()


def move(robot, arguments):
    """串行移动到一个或多个预设位姿。只允许管理员配置的编号1、2、3；完成前不得提交其他动作。"""
    if not isinstance(arguments, dict) or not set(arguments).issubset({"pose_ids", "options"}) or "pose_ids" not in arguments:
        raise ValueError("参数必须包含 pose_ids，可选 options")
    pose_ids = arguments["pose_ids"]
    if not isinstance(pose_ids, list) or not pose_ids or any(type(x) is not int or x not in {1, 2, 3} for x in pose_ids):
        raise ValueError("pose_ids 必须是由1、2、3组成的非空数组")
    options = _options(arguments.get("options"))
    return robot.move_pose_ids(pose_ids, options)


def grasp(robot, arguments):
    """执行已标定 AprilTag 的完整抓放任务。tag_id 与 offset_config 必须由用户或任务配置明确提供，禁止猜测。"""
    if not isinstance(arguments, dict) or not set(arguments).issubset({"tag_id", "offset_config", "options"}):
        raise ValueError("参数只能包含 tag_id、offset_config、options")
    if type(arguments.get("tag_id")) is not int:
        raise ValueError("tag_id 必须是整数")
    offset = arguments.get("offset_config")
    if not isinstance(offset, str) or not offset.strip() or "/" in offset or "\\" in offset or not offset.endswith(".yaml"):
        raise ValueError("offset_config 必须是明确的 yaml 文件名，不能包含路径")
    options = _options(arguments.get("options"), MOVE_OPTIONS - {"pause-s"})
    return robot.grasp_apriltag(arguments["tag_id"], offset, options)


def cancel(robot, arguments):
    """紧急请求取消当前机器人动作。HTTP 202 仅表示已请求取消，后续必须检查 busy=false。"""
    if not isinstance(arguments, dict) or arguments:
        raise ValueError("cancel_robot 不接受参数")
    return robot.cancel()


HEALTH_TOOL = {"type": "function", "function": {"name": "robot_health", "description": health.__doc__, "parameters": {"type": "object", "properties": {}, "additionalProperties": False}}}
MOVE_TOOL = {"type": "function", "function": {"name": "move_robot", "description": move.__doc__, "parameters": {"type": "object", "properties": {"pose_ids": {"type": "array", "items": {"type": "integer", "enum": [1, 2, 3]}, "minItems": 1}, "options": {"type": "object"}}, "required": ["pose_ids"], "additionalProperties": False}}}
GRASP_TOOL = {"type": "function", "function": {"name": "grasp_apriltag", "description": grasp.__doc__, "parameters": {"type": "object", "properties": {"tag_id": {"type": "integer"}, "offset_config": {"type": "string"}, "options": {"type": "object"}}, "required": ["tag_id", "offset_config"], "additionalProperties": False}}}
CANCEL_TOOL = {"type": "function", "function": {"name": "cancel_robot", "description": cancel.__doc__, "parameters": {"type": "object", "properties": {}, "additionalProperties": False}}}

ROBOT_TOOL_SCHEMAS = [HEALTH_TOOL, MOVE_TOOL, GRASP_TOOL, CANCEL_TOOL]
ROBOT_TOOL_HANDLERS = {"robot_health": health, "move_robot": move, "grasp_apriltag": grasp, "cancel_robot": cancel}
