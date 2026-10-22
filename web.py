"""零依赖的实时网页服务。"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
import uuid
from urllib.parse import urlparse

from config.settings import DEFAULT_MAX_STEPS, RUNS_DIR
from harness.model import ChatClient
from harness.runtime import Harness
from robot.client import RobotClient
from robot.http_client import RealRobotClient
from tools import ROBOT_TOOL_SCHEMAS


WEB_DIR = Path(__file__).with_name("web_static")


class WebRuntime:
    def __init__(self, max_steps, mode="virtual"):
        self.mode = mode
        self.backend = RealRobotClient() if mode == "robot" else RobotClient()
        self.game = self.backend
        tools = ROBOT_TOOL_SCHEMAS if mode == "robot" else None
        self.client = ChatClient(tools)
        log_path = Path(RUNS_DIR) / (uuid.uuid4().hex + ".jsonl")
        prompt = "prompt_robot.md" if mode == "robot" else "prompt.md"
        self.harness = Harness(self.client, self.backend, log_path, max_steps, prompt)
        self.lock = threading.Lock()
        self.running = False
        self.phase = "ready"
        self.answer = "等待任务"
        self.error = ""
        self.started_at = None
        self.health_cache = None
        self.health_cache_time = 0

    def snapshot(self):
        with self.lock:
            agent = {"running": self.running, "phase": self.phase, "answer": self.answer,
                     "error": self.error, "started_at": self.started_at}
        if self.mode == "robot":
            if time.time() - self.health_cache_time > 1:
                self.health_cache = self.backend.health()
                self.health_cache_time = time.time()
            return {"mode": "robot", "robot": self.health_cache, "agent": agent,
                    "events": self.backend.recent_events(), "base_url": self.backend.base_url}
        return {"mode": "virtual", "game": self.game.observe_game(), "agent": agent,
                "events": self.game.recent_events()}

    def start(self, request):
        request = request.strip()
        if not request:
            raise ValueError("任务不能为空")
        with self.lock:
            if self.running:
                raise RuntimeError("智能体正在执行任务")
            self.running = True
            self.phase = "thinking"
            self.answer = "模型正在观察局面并规划…"
            self.error = ""
            self.started_at = time.time()
        threading.Thread(target=self._run, args=(request,), daemon=True).start()

    def _run(self, request):
        try:
            answer = self.harness.run(request)
            with self.lock:
                self.answer = answer
                self.phase = "finished"
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
                self.answer = "任务执行失败"
                self.phase = "error"
        finally:
            with self.lock:
                self.running = False


def make_handler(runtime):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            return

        def send_json(self, value, status=200):
            data = json.dumps(value, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def read_json(self):
            size = int(self.headers.get("Content-Length", "0"))
            if size > 16_384:
                raise ValueError("请求内容过大")
            return json.loads(self.rfile.read(size) or b"{}")

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/api/state":
                self.send_json(runtime.snapshot())
                return
            filename = ("robot.html" if runtime.mode == "robot" else "index.html") if path == "/" else path.lstrip("/")
            if filename not in {"index.html", "app.js", "styles.css", "robot.html", "robot.js", "robot.css"}:
                self.send_error(404)
                return
            data = WEB_DIR.joinpath(filename).read_bytes()
            mime = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
                    ".css": "text/css; charset=utf-8"}[Path(filename).suffix]
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_POST(self):
            try:
                body = self.read_json()
                if self.path == "/api/task":
                    runtime.start(str(body.get("request", "")))
                    self.send_json({"ok": True}, 202)
                elif self.path == "/api/action":
                    if runtime.mode != "virtual":
                        raise RuntimeError("真机模式不提供手动游戏控制")
                    if runtime.running:
                        raise RuntimeError("智能体运行时不能手动控制")
                    self.send_json(runtime.game.move_ball(str(body.get("direction", ""))))
                elif self.path == "/api/reset":
                    if runtime.mode != "virtual":
                        raise RuntimeError("真机模式不提供游戏重置")
                    if runtime.running:
                        raise RuntimeError("智能体运行时不能重置")
                    runtime.harness.messages = [{"role": "system", "content": runtime.harness.system_prompt}]
                    with runtime.lock:
                        runtime.phase, runtime.answer, runtime.error = "ready", "等待任务", ""
                    self.send_json(runtime.game.reset_game())
                elif self.path == "/api/cancel":
                    if runtime.mode != "robot":
                        raise RuntimeError("仅真机模式支持取消")
                    self.send_json(runtime.backend.cancel())
                else:
                    self.send_json({"error": "not_found"}, 404)
            except (ValueError, RuntimeError, json.JSONDecodeError) as exc:
                self.send_json({"error": str(exc)}, 409)

    return Handler


def run_web(host="0.0.0.0", port=8080, max_steps=DEFAULT_MAX_STEPS, mode="virtual"):
    runtime = WebRuntime(max_steps, mode)
    server = ThreadingHTTPServer((host, port), make_handler(runtime))
    print(f"{('真实机器人' if mode == 'robot' else '虚拟游戏')}网页已启动：http://127.0.0.1:{port}")
    print(f"局域网访问：http://<本机IP>:{port}（服务监听 {host}）")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n网页服务已停止。")
    finally:
        server.server_close()


def main():
    parser = argparse.ArgumentParser(description="大模型实时控制虚拟小球网页")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--max-steps", type=int, default=DEFAULT_MAX_STEPS)
    parser.add_argument("--mode", choices=["virtual", "robot"], default="virtual")
    args = parser.parse_args()
    run_web(args.host, args.port, args.max_steps, args.mode)


if __name__ == "__main__":
    main()
