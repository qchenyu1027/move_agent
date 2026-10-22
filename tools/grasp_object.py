"""抓取指定物体，不隐式执行固定位置移动。"""


def execute(robot, arguments):
    """请求机器人抓取指定物体。

    用户明确要求“抓取”“拿起”且物体描述清楚时调用。
    target 是自然语言目标描述，例如“桌上的水”，保留用户提供的特征。
    无法从当前输入和对话确定目标时先澄清；普通聊天或询问操作方法时不调用。
    不猜测物体坐标、ID、存在性或可达性，后端负责感知、抓取规划和执行验证。
    本工具不调用固定位置移动，不包含底盘导航、递交、放置或松开；抓取所需机械臂动作由后端完成。
    若任务要求先移动到固定位置，应先调用 move_to_fixed_position，确认成功后再调用本工具。
    ok=true 才表示后端确认抓取成功；retryable=false 时停止自动尝试。
    """
    if not isinstance(arguments, dict) or set(arguments) != {"target"}:
        raise ValueError('参数只能包含 target，例如 {"target": "红色玩具"}')
    target = arguments["target"]
    if not isinstance(target, str) or not target.strip() or len(target) > 200:
        raise ValueError("target 必须是1至200字符的非空目标描述")
    return robot.grasp_object(target=target.strip())


TOOL = {
    "type": "function",
    "function": {
        "name": "grasp_object",
        "description": execute.__doc__.strip(),
        "parameters": {
            "type": "object",
            "properties": {"target": {
                "type": "string", "description": "用户要求抓取的物体描述，例如桌上的水瓶；不要编造坐标或ID。",
                "minLength": 1, "maxLength": 200,
            }},
            "required": ["target"], "additionalProperties": False,
        },
    },
}
