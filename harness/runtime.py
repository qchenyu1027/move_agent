"""任务循环、日志与组件协调。"""
import json
from pathlib import Path
import re
import time
from tools import execute_tool


_DIRECTION_MAP = {"上": "up", "下": "down", "左": "left", "右": "right"}
_CHINESE_NUMBERS = {"一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5,
                    "六": 6, "七": 7, "八": 8, "九": 9, "十": 10}


def explicit_move_plan(request):
    """提取用户明确指定的方向步数；目标型任务返回空列表。"""
    pattern = r"向?([上下左右])(?:方向)?(?:移动|走)?\s*([一二两三四五六七八九十\d]+)\s*步"
    plan = []
    for direction, count_text in re.findall(pattern, request):
        if count_text.isdigit():
            count = int(count_text)
        else:
            count = _CHINESE_NUMBERS.get(count_text)
        if count is not None and 1 <= count <= 100:
            plan.extend([_DIRECTION_MAP[direction]] * count)
    return plan


class Harness:
    def __init__(self, client, robot, log_path, max_steps=6, prompt_file="prompt.md"):
        self.client = client
        self.robot = robot
        self.log_path = Path(log_path)
        self.max_steps = max_steps
        prompt = Path(__file__).resolve().parents[1].joinpath(prompt_file).read_text(encoding="utf-8")
        self.system_prompt = prompt
        self.messages = [{"role": "system", "content": self.system_prompt}]

    def record(self, event, **data):
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        with self.log_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"time": time.time(), "event": event, **data}, ensure_ascii=False) + "\n")

    def execute(self, name, arguments):
        return execute_tool(self.robot, name, arguments)

    def run(self, request):
        messages = list(self.messages)
        messages.append({"role": "user", "content": request})
        self.record("task_started", request=request)
        move_plan = explicit_move_plan(request)
        moves_done = 0
        try:
            for step in range(self.max_steps):
                reply = self.client.complete(messages)
                messages.append(reply)
                self.record("model_message", step=step, message=reply)
                calls = reply.get("tool_calls") or []
                if not calls:
                    answer = reply.get("content") or "模型未返回回答。"
                    self.record("task_finished")
                    self.messages = messages
                    return answer
                stop_reason = None
                last_move_result = None
                for call in calls:
                    function = call["function"]
                    try:
                        args = json.loads(function["arguments"])
                        if stop_reason is not None:
                            result = {"ok": False, "error": "batch_cancelled", "retryable": False,
                                      "message": stop_reason}
                        elif function["name"] == "move_ball" and move_plan:
                            if moves_done >= len(move_plan):
                                result = {"ok": False, "error": "requested_steps_completed", "retryable": False,
                                          "message": "用户指定的移动步数已经完成，禁止继续移动。"}
                            elif args.get("direction") != move_plan[moves_done]:
                                result = {"ok": False, "error": "direction_not_requested", "retryable": False,
                                          "message": f"用户要求第{moves_done + 1}步向{move_plan[moves_done]}，禁止改为其他方向。"}
                            else:
                                result = self.execute(function["name"], args)
                                moves_done += 1
                                last_move_result = result
                        else:
                            result = self.execute(function["name"], args)
                    except (ValueError, TypeError) as exc:
                        result = {"ok": False, "error": str(exc)}
                    if result.get("ok") is False and result.get("retryable") is False:
                        stop_reason = result.get("message") or result.get("error") or "工具执行失败，停止重试。"
                    self.record("tool_result", tool=function["name"], result=result)
                    messages.append({"role": "tool", "tool_call_id": call["id"],
                                     "content": json.dumps(result, ensure_ascii=False)})
                if move_plan and moves_done == len(move_plan):
                    position = (last_move_result or self.robot.observe_game()).get("ball", {})
                    answer = f"已严格按要求完成{moves_done}步移动，当前位置：({position.get('x')}, {position.get('y')})。"
                    messages.append({"role": "assistant", "content": answer})
                    self.messages = messages
                    self.record("task_finished", reason="explicit_move_plan_completed", moves=moves_done)
                    return answer
                if stop_reason is not None:
                    messages.append({"role": "assistant", "content": stop_reason})
                    self.messages = messages
                    self.record("task_failed", error=stop_reason)
                    return stop_reason
            raise RuntimeError("达到最大执行轮数，任务已停止")
        except Exception as exc:
            self.record("task_failed", error_type=type(exc).__name__)
            raise
