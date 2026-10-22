"""真实机器人 HTTP API 客户端。所有动作串行，且绝不自动重试。"""
from collections import deque
import json
import os
import threading
import time
import urllib.error
import urllib.request


class RealRobotClient:
    def __init__(self, base_url=None, api_key=None):
        self.base_url = (base_url or os.getenv("ROBOT_API_BASE", "http://10.121.40.94:8000")).rstrip("/")
        self.api_key = api_key if api_key is not None else os.getenv("ROBOT_API_KEY", "")
        if not self.api_key:
            raise ValueError("真机模式需要设置 ROBOT_API_KEY 环境变量")
        self.health_timeout = float(os.getenv("ROBOT_HEALTH_TIMEOUT_S", "5"))
        self.action_timeout = float(os.getenv("ROBOT_ACTION_TIMEOUT_S", "600"))
        self.lock = threading.Lock()
        self.events = deque(maxlen=50)
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self._event("ready", "真机 HTTP 客户端已配置", base_url=self.base_url)

    def _event(self, kind, message, **data):
        self.events.appendleft({"time": time.time(), "kind": kind, "message": message, **data})

    def recent_events(self):
        return list(self.events)

    def _request(self, method, path, payload=None, timeout=None, auth=False):
        headers = {"Accept": "application/json"}
        if payload is not None:
            headers["Content-Type"] = "application/json"
        if auth:
            headers["X-API-Key"] = self.api_key
        request = urllib.request.Request(self.base_url + path,
            data=None if payload is None else json.dumps(payload).encode("utf-8"),
            headers=headers, method=method)
        try:
            with self.opener.open(request, timeout=timeout) as response:
                body = json.load(response)
                body["http_status"] = response.status
                return body
        except urllib.error.HTTPError as exc:
            try:
                body = json.loads(exc.read().decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                body = {"message": "机器人服务返回非 JSON 错误"}
            return {"ok": False, "http_status": exc.code, "retryable": False, **body}
        except (urllib.error.URLError, TimeoutError, OSError):
            return {"ok": False, "error": "connection_or_timeout", "retryable": False,
                    "result_unknown": method != "GET",
                    "message": "机器人连接失败或请求超时；动作结果未知，禁止自动重试。"}

    def health(self):
        result = self._request("GET", "/health", timeout=self.health_timeout)
        self._event("health" if result.get("ok") else "error",
                    "机器人健康检查通过" if result.get("ok") else result.get("message", "机器人健康检查失败"),
                    ready_for_motion=result.get("ready_for_motion"), busy=result.get("api", {}).get("busy"))
        return result

    def _action(self, path, payload, label):
        with self.lock:
            health = self.health()
            if health.get("http_status") != 200 or not health.get("ok") or not health.get("ready_for_motion"):
                return {"ok": False, "error": "robot_not_ready", "retryable": False,
                        "message": "机器人未满足运动准入条件，未提交动作。", "health": health}
            self._event("action", f"开始执行：{label}", request=payload)
            result = self._request("POST", path, payload, timeout=self.action_timeout, auth=True)
            if result.get("http_status") == 200 and result.get("ok") is True:
                self._event("success", f"执行完成：{label}")
                return result
            result.setdefault("retryable", False)
            result.setdefault("message", f"动作失败：{label}；禁止自动重试。")
            self._event("error", result["message"], http_status=result.get("http_status"))
            return result

    def move_pose_ids(self, pose_ids, options=None):
        payload = {"pose_ids": pose_ids}
        if options:
            payload["options"] = options
        return self._action("/move", payload, f"移动到预设位姿 {pose_ids}")

    def grasp_apriltag(self, tag_id, offset_config, options=None):
        merged = dict(options or {})
        merged["offset-config"] = offset_config
        return self._action("/grasp", {"tag_id": tag_id, "options": merged}, f"AprilTag {tag_id} 抓放")

    def cancel(self):
        result = self._request("POST", "/cancel", timeout=self.health_timeout, auth=True)
        accepted = result.get("http_status") == 202
        self._event("cancel", "取消请求已提交，等待机器人清理" if accepted else result.get("message", "取消失败"))
        result["accepted"] = accepted
        return result
