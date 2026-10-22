"""读取当前虚拟游戏状态。"""


def execute(robot, arguments):
    """观察小球、终点、障碍、边界、分数和游戏棋盘；规划前或不确定局面时调用。"""
    if not isinstance(arguments, dict) or arguments:
        raise ValueError("observe_game 不接受参数，请传空对象 {}")
    return robot.observe_game()


TOOL = {"type": "function", "function": {"name": "observe_game", "description": execute.__doc__,
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False}}}
