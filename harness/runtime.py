"""单次 LLM 路由：选择一个机器人动作，或者直接回答。"""
import json
from pathlib import Path
import time

from tools import execute_tool


class Harness:
    def __init__(self, client, robot, log_path, prompt_file="prompt_robot.md"):
        self.client = client
        self.robot = robot
        self.log_path = Path(log_path)
        self.system_prompt = (
            Path(__file__).resolve().parents[1]
            .joinpath(prompt_file)
            .read_text(encoding="utf-8")
        )

    def record(self, event, **data):
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        with self.log_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(
                {"time": time.time(), "event": event, **data},
                ensure_ascii=False,
            ) + "\n")

    def run(self, request):
        messages = [
            {"role": "system", "content": self.system_prompt},
            {"role": "user", "content": request},
        ]
        self.record("task_started", request=request)
        try:
            reply = self.client.complete(messages)
            if not isinstance(reply, dict):
                raise RuntimeError("模型响应格式错误")
            self.record("model_message", message=reply)
            calls = reply.get("tool_calls") or []
            if not isinstance(calls, list):
                raise RuntimeError("模型工具调用格式错误")
            if not calls:
                answer = reply.get("content") or "请换一种方式描述问题。"
                self.record("task_finished", mode="answer")
                return answer
            if len(calls) != 1:
                answer = "一次只能执行一个机器人动作，请分别下达指令。"
                self.record("task_failed", error="multiple_tool_calls", count=len(calls))
                return answer

            call = calls[0]
            try:
                function = call["function"]
                name = function["name"]
                arguments = json.loads(function.get("arguments") or "{}")
                result = execute_tool(self.robot, name, arguments)
                if not isinstance(result, dict):
                    raise TypeError("工具必须返回对象")
            except (KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
                self.record("task_failed", error="invalid_tool_call", detail=str(exc))
                return "工具参数无效，未执行机器人动作。"

            self.record("tool_result", tool=name, result=result)
            if result.get("ok") is True:
                self.record("task_finished", mode="action")
                return result.get("message") or "机器人动作执行完成。"
            answer = result.get("message") or result.get("error") or "机器人动作执行失败。"
            self.record("task_failed", error=answer)
            return answer
        except Exception as exc:
            self.record("task_failed", error_type=type(exc).__name__)
            raise
