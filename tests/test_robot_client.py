import json
import urllib.error
from robot.http_client import RealRobotClient
from tools.robot_tools import move


class Response:
    def __init__(self, body, status=200): self.body, self.status = body, status
    def __enter__(self): return self
    def __exit__(self, *args): return False
    def read(self): return json.dumps(self.body).encode()


class Opener:
    def __init__(self, responses): self.responses, self.requests = list(responses), []
    def open(self, request, timeout=None):
        self.requests.append((request, timeout))
        response = self.responses.pop(0)
        if isinstance(response, Exception): raise response
        return response


def client(responses):
    result = RealRobotClient("http://robot:8000", "secret")
    result.opener = Opener(responses)
    return result


def test_move_checks_health_then_sends_authenticated_action():
    robot = client([Response({"ok": True, "ready_for_motion": True, "api": {"busy": False}}), Response({"ok": True})])
    result = robot.move_pose_ids([3])
    assert result["ok"] is True
    assert [r.full_url for r, _ in robot.opener.requests] == ["http://robot:8000/health", "http://robot:8000/move"]
    action = robot.opener.requests[1][0]
    assert action.headers["X-api-key"] == "secret"
    assert json.loads(action.data) == {"pose_ids": [3]}


def test_not_ready_never_submits_action():
    robot = client([Response({"ok": True, "ready_for_motion": False, "api": {"busy": True}})])
    result = robot.move_pose_ids([1])
    assert result["error"] == "robot_not_ready"
    assert len(robot.opener.requests) == 1


def test_timeout_is_unknown_and_not_retryable():
    robot = client([Response({"ok": True, "ready_for_motion": True}), urllib.error.URLError("timeout")])
    result = robot.move_pose_ids([2])
    assert result["result_unknown"] is True
    assert result["retryable"] is False
    assert len(robot.opener.requests) == 2


def test_tool_rejects_unapproved_pose_and_option():
    robot = object()
    try:
        move(robot, {"pose_ids": [9]})
        assert False
    except ValueError: pass
    try:
        move(robot, {"pose_ids": [1], "options": {"unsafe": 1}})
        assert False
    except ValueError: pass
