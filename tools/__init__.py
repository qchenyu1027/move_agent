"""虚拟游戏工具注册表。"""
from .observe_game import TOOL as OBSERVE_TOOL, execute as execute_observe
from .move_ball import TOOL as MOVE_TOOL, execute as execute_move
from .reset_game import TOOL as RESET_TOOL, execute as execute_reset
from .robot_tools import ROBOT_TOOL_SCHEMAS, ROBOT_TOOL_HANDLERS

TOOL_SCHEMAS = [OBSERVE_TOOL, MOVE_TOOL, RESET_TOOL]
TOOL_HANDLERS = {"observe_game": execute_observe, "move_ball": execute_move, "reset_game": execute_reset}
TOOL_HANDLERS.update(ROBOT_TOOL_HANDLERS)


def execute_tool(robot, name, arguments):
    handler = TOOL_HANDLERS.get(name)
    if handler is None:
        raise ValueError("tool_not_allowed")
    return handler(robot, arguments)
