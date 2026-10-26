import json

import pytest

from harness.runtime import Harness
from tools.robot_tools import grasp, move


class FakeRobot:
    def __init__(self):
        self.actions = []

    def move_pose_ids(self, pose_ids):
        assert all(type(value) is int for value in pose_ids)
        self.actions.append(("move", pose_ids))
        return {"ok": True, "message": "移动完成"}

    def grasp_apriltag(self, tag_id, offset_config):
        assert type(tag_id) is int
        self.actions.append(("grasp", tag_id, offset_config))
        return {"ok": True, "message": "抓取完成"}


@pytest.mark.parametrize("value", [1, 2, 3, "1", "2", "3", " 2\n"])
def test_move_normalizes_allowed_ids(value):
    robot = FakeRobot()
    assert move(robot, {"pose_id": value})["ok"] is True
    assert robot.actions == [("move", [int(value)])]


@pytest.mark.parametrize("value", [3, 5, "3", "5", " 5\n"])
def test_grasp_normalizes_allowed_ids_and_selects_calibration(value):
    robot = FakeRobot()
    assert grasp(robot, {"tag_id": value})["ok"] is True
    tag_id = int(value)
    assert robot.actions == [("grasp", tag_id, f"tag{tag_id}_left_tool_offset.yaml")]


@pytest.mark.parametrize("action, field", [(move, "pose_id"), (grasp, "tag_id")])
@pytest.mark.parametrize("value", [
    True, False, None, 1.0, 3.0, 5.5, 9, "9", "", " ",
    "3.0", "3e0", "+3", "03", "３", "三", "3; echo test", [], {},
])
def test_invalid_ids_never_reach_robot(action, field, value):
    robot = FakeRobot()
    with pytest.raises(ValueError):
        action(robot, {field: value})
    assert robot.actions == []


@pytest.mark.parametrize("action, arguments", [
    (move, {}),
    (move, {"pose_id": "2", "speed": 1}),
    (grasp, {"tag_id": "5", "offset-config": "other.yaml"}),
    (grasp, None),
])
def test_argument_structure_remains_restricted(action, arguments):
    robot = FakeRobot()
    with pytest.raises(ValueError):
        action(robot, arguments)
    assert robot.actions == []


@pytest.mark.parametrize("name, arguments, expected", [
    ("move_robot", {"pose_id": "2"}, ("move", [2])),
    ("grasp_object", {"tag_id": "3"}, ("grasp", 3, "tag3_left_tool_offset.yaml")),
])
def test_model_string_ids_work_through_harness(tmp_path, name, arguments, expected):
    class ModelReply:
        def complete(self, messages):
            return {"role": "assistant", "content": None, "tool_calls": [{
                "id": "call-1",
                "type": "function",
                "function": {"name": name, "arguments": json.dumps(arguments)},
            }]}

    robot = FakeRobot()
    answer = Harness(ModelReply(), robot, tmp_path / "task.jsonl").run("执行任务")
    assert answer in {"移动完成", "抓取完成"}
    assert robot.actions == [expected]
