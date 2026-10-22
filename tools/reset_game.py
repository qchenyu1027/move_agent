"""重置虚拟游戏。"""


def execute(robot, arguments):
    """仅当用户明确要求重新开始、重置或开启新一局时调用。"""
    if not isinstance(arguments, dict) or arguments:
        raise ValueError("reset_game 不接受参数，请传空对象 {}")
    return robot.reset_game()


TOOL = {"type": "function", "function": {"name": "reset_game", "description": execute.__doc__,
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False}}}
