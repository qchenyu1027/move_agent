"""真实机器人 LLM 工具注册表。"""
from .robot_tools import ROBOT_TOOL_HANDLERS, ROBOT_TOOL_SCHEMAS


TOOL_SCHEMAS = ROBOT_TOOL_SCHEMAS
TOOL_HANDLERS = dict(ROBOT_TOOL_HANDLERS)


def execute_tool(robot, name, arguments):
    handler = TOOL_HANDLERS.get(name)
    if handler is None:
        raise ValueError("tool_not_allowed")
    return handler(robot, arguments)
