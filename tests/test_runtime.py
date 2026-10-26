from harness.runtime import Harness


def tool_call(call_id, name, arguments):
    return {"id": call_id, "function": {"name": name, "arguments": arguments}}


class ScriptedClient:
    def __init__(self, replies):
        self.replies = list(replies)
        self.seen_messages = []

    def complete(self, messages):
        self.seen_messages.append(list(messages))
        return self.replies.pop(0)


class FakeRobot:
    def __init__(self):
        self.moves = []
        self.grasps = []

    def move_pose_ids(self, pose_ids, options=None):
        self.moves.append((pose_ids, options))
        return {"ok": True, "message": "移动完成"}

    def grasp_apriltag(self, tag_id, offset_config, options=None):
        self.grasps.append((tag_id, offset_config, options))
        return {"ok": True, "message": "抓取完成"}


def test_move_uses_one_model_call(tmp_path):
    client = ScriptedClient([{
        "role": "assistant",
        "content": None,
        "tool_calls": [tool_call("move", "move_robot", '{"pose_id":2}')],
    }])
    robot = FakeRobot()
    harness = Harness(client, robot, tmp_path / "move.jsonl")
    assert harness.run("移动到2") == "移动完成"
    assert robot.moves == [([2], None)]
    assert len(client.seen_messages) == 1


def test_default_grasp_uses_tag5_calibration(tmp_path):
    client = ScriptedClient([{
        "role": "assistant",
        "content": None,
        "tool_calls": [tool_call("grasp", "grasp_object", "{}")],
    }])
    robot = FakeRobot()
    harness = Harness(client, robot, tmp_path / "grasp.jsonl")
    assert harness.run("抓住这个东西") == "抓取完成"
    assert robot.grasps == [(5, "tag5_left_tool_offset.yaml", None)]


def test_plain_question_returns_model_answer_without_robot_action(tmp_path):
    client = ScriptedClient([{"role": "assistant", "content": "你好，有什么可以帮你？"}])
    robot = FakeRobot()
    harness = Harness(client, robot, tmp_path / "answer.jsonl")
    assert harness.run("你好") == "你好，有什么可以帮你？"
    assert robot.moves == []
    assert robot.grasps == []


def test_multiple_tool_calls_are_rejected_before_motion(tmp_path):
    client = ScriptedClient([{
        "role": "assistant",
        "content": None,
        "tool_calls": [
            tool_call("move-1", "move_robot", '{"pose_id":1}'),
            tool_call("move-2", "move_robot", '{"pose_id":2}'),
        ],
    }])
    robot = FakeRobot()
    answer = Harness(client, robot, tmp_path / "batch.jsonl").run("依次移动")
    assert "一次只能" in answer
    assert robot.moves == []


def test_each_user_request_has_independent_context(tmp_path):
    client = ScriptedClient([
        {"role": "assistant", "content": "第一项回答"},
        {"role": "assistant", "content": "第二项回答"},
    ])
    harness = Harness(client, FakeRobot(), tmp_path / "tasks.jsonl")
    assert harness.run("第一项") == "第一项回答"
    assert harness.run("第二项") == "第二项回答"
    assert [messages[1]["content"] for messages in client.seen_messages] == ["第一项", "第二项"]
    assert all(len(messages) == 2 for messages in client.seen_messages)


def test_unknown_tool_stops_without_motion(tmp_path):
    client = ScriptedClient([{
        "role": "assistant",
        "content": None,
        "tool_calls": [tool_call("bad", "unknown_tool", "{}")],
    }])
    robot = FakeRobot()
    answer = Harness(client, robot, tmp_path / "unknown.jsonl").run("执行未知工具")
    assert "参数无效" in answer
    assert robot.moves == []
