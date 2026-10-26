import json
import urllib.error

from robot.http_client import RealRobotClient
from tools.robot_tools import grasp, move


class Response:
    def __init__(self, body, status=200):
        self.body, self.status = body, status

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def read(self):
        return json.dumps(self.body).encode()


class Opener:
    def __init__(self, responses):
        self.responses, self.requests = list(responses), []

    def open(self, request, timeout=None):
        self.requests.append((request, timeout))
        response = self.responses.pop(0)
        if isinstance(response, Exception):
            raise response
        return response


def client(responses):
    result = RealRobotClient("http://robot:8000", "secret")
    result.opener = Opener(responses)
    return result


def test_move_checks_health_then_sends_authenticated_action():
    robot = client([
        Response({"ok": True, "ready_for_motion": True, "api": {"busy": False}}),
        Response({"ok": True}),
    ])
    result = robot.move_pose_ids([3])
    assert result["ok"] is True
    assert [r.full_url for r, _ in robot.opener.requests] == [
        "http://robot:8000/health", "http://robot:8000/move",
    ]
    action = robot.opener.requests[1][0]
    assert action.headers["X-api-key"] == "secret"
    assert json.loads(action.data) == {"pose_ids": [3]}


def test_not_ready_never_submits_action():
    robot = client([Response({"ok": True, "ready_for_motion": False, "api": {"busy": True}})])
    result = robot.move_pose_ids([1])
    assert result["error"] == "robot_not_ready"
    assert len(robot.opener.requests) == 1


def test_timeout_is_unknown_and_not_retryable():
    robot = client([
        Response({"ok": True, "ready_for_motion": True}),
        urllib.error.URLError("timeout"),
    ])
    result = robot.move_pose_ids([2])
    assert result["result_unknown"] is True
    assert result["retryable"] is False
    assert len(robot.opener.requests) == 2


def test_llm_tools_use_simple_arguments():
    class FakeRobot:
        def move_pose_ids(self, pose_ids, options=None):
            return {"pose_ids": pose_ids}

        def grasp_apriltag(self, tag_id, offset_config, options=None):
            return {"tag_id": tag_id, "offset_config": offset_config}

    robot = FakeRobot()
    assert move(robot, {"pose_id": 2}) == {"pose_ids": [2]}
    assert grasp(robot, {}) == {
        "tag_id": 5,
        "offset_config": "tag5_left_tool_offset.yaml",
    }


def test_tools_reject_unapproved_values():
    robot = object()
    for action, arguments in ((move, {"pose_id": 9}), (grasp, {"tag_id": 7})):
        try:
            action(robot, arguments)
            assert False
        except ValueError:
            pass
