"""线程安全的虚拟游戏后端；保留 RobotClient 名称以兼容现有 Harness。"""
from collections import deque
import os
import threading
import time
from game import BallGame


class RobotClient:
    def __init__(self):
        self.game = BallGame()
        self.lock = threading.RLock()
        self.events = deque(maxlen=30)
        self.delay = float(os.getenv("MOVE_AGENT_ACTION_DELAY", "0.25"))
        self._event("reset", "虚拟环境已就绪")

    def _event(self, kind, message, **data):
        self.events.appendleft({"time": time.time(), "kind": kind, "message": message, **data})

    def recent_events(self):
        with self.lock:
            return list(self.events)

    def observe_game(self) -> dict:
        with self.lock:
            return self.game.observe()

    def move_ball(self, direction: str) -> dict:
        with self.lock:
            result = self.game.move(direction)
            self._event("move", result["message"], direction=direction, blocked=result["blocked"],
                        position=result["ball"], score=result["score"])
        if self.delay > 0:
            time.sleep(self.delay)
        return result

    def reset_game(self) -> dict:
        with self.lock:
            result = self.game.reset()
            result["message"] = "游戏已重置。"
            self.events.clear()
            self._event("reset", result["message"])
            return result
