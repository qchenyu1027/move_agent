from pathlib import Path
from game import BallGame
from harness.runtime import Harness, explicit_move_plan
from robot.client import RobotClient


class ScriptedClient:
    def __init__(self): self.calls = 0
    def complete(self, messages):
        self.calls += 1
        if self.calls == 1:
            return {"role": "assistant", "content": None, "tool_calls": [{"id": "observe", "function": {"name": "observe_game", "arguments": "{}"}}]}
        return {"role": "assistant", "content": None, "tool_calls": [{"id": "move", "function": {"name": "move_ball", "arguments": '{"direction":"left"}'}}]}


def test_explicit_move_plan():
    assert explicit_move_plan("观察游戏，把小球向左移动一步") == ["left"]
    assert explicit_move_plan("向上移动2步，再向右走两步") == ["up", "up", "right", "right"]
    assert explicit_move_plan("把小球移动到终点") == []


def test_one_step_request_stops_immediately(tmp_path):
    robot = RobotClient(); robot.delay = 0
    client = ScriptedClient()
    harness = Harness(client, robot, tmp_path / "run.jsonl", max_steps=10)
    answer = harness.run("观察游戏，把小球向左移动一步")
    assert robot.observe_game()["ball"] == {"x": 0, "y": 6}
    assert robot.observe_game()["steps"] == 1
    assert client.calls == 2
    assert "完成1步" in answer


class ManyCallsClient:
    def __init__(self): self.calls = 0
    def complete(self, messages):
        self.calls += 1
        if self.calls == 1:
            calls = [{"id": f"move-{i}", "function": {"name": "move_ball", "arguments": '{"direction":"right"}'}} for i in range(6)]
            return {"role": "assistant", "content": None, "tool_calls": calls}
        return {"role": "assistant", "content": "已完成"}


def test_more_than_four_tool_calls_are_allowed(tmp_path):
    robot = RobotClient(); robot.delay = 0
    harness = Harness(ManyCallsClient(), robot, tmp_path / "many.jsonl", max_steps=3)
    assert harness.run("自由移动") == "已完成"
    assert robot.observe_game()["steps"] == 6
