"""在虚拟游戏中移动小球。"""


def execute(robot, arguments):
    """让小球移动一格。每次只能选择 up、down、left、right 之一；根据返回的新状态继续规划。"""
    if not isinstance(arguments, dict) or set(arguments) != {"direction"}:
        raise ValueError('参数必须是 {"direction": "up|down|left|right"}')
    return robot.move_ball(arguments["direction"])


TOOL = {"type": "function", "function": {"name": "move_ball", "description": execute.__doc__,
        "parameters": {"type": "object", "properties": {"direction": {"type": "string", "enum": ["up", "down", "left", "right"]}},
                       "required": ["direction"], "additionalProperties": False}}}
