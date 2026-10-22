"""移动到机器人端配置的固定位置。"""


def execute(robot, arguments):
    """将机器人移动到后端预先配置的固定位置。

    用户明确要求“移动到固定位置”“回到预设位置”时调用。
    此工具没有参数，固定目标位姿由机器人端配置，不让模型生成坐标。
    只执行移动，不执行抓取、放置或松开。
    用户指定了其他位置时先澄清，本工具不能移动到任意位置。
    ok=true 才表示后端确认到达；retryable=false 时停止自动尝试。
    """
    if not isinstance(arguments, dict) or arguments:
        raise ValueError("move_to_fixed_position 不接受参数，请传空对象 {}")
    return robot.move_to_fixed_position()


TOOL = {
    "type": "function",
    "function": {
        "name": "move_to_fixed_position",
        "description": execute.__doc__.strip(),
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
    },
}
