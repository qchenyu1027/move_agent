"""轻量、无依赖的网格小球游戏环境。"""


class BallGame:
    WIDTH = 12
    HEIGHT = 8
    START = (1, 6)
    GOAL = (10, 1)
    OBSTACLES = frozenset({(3, 1), (3, 2), (3, 3), (3, 4), (6, 3), (6, 4), (6, 5), (8, 1), (8, 2), (8, 3)})
    DELTAS = {"up": (0, -1), "down": (0, 1), "left": (-1, 0), "right": (1, 0)}

    def __init__(self):
        self.episode = 0
        self.reset()

    def reset(self):
        self.episode += 1
        self.ball = self.START
        self.steps = 0
        self.score = 0
        self.done = False
        return self.observe()

    def observe(self):
        return {
            "ok": True,
            "episode": self.episode,
            "ball": {"x": self.ball[0], "y": self.ball[1]},
            "goal": {"x": self.GOAL[0], "y": self.GOAL[1]},
            "obstacles": [{"x": x, "y": y} for x, y in sorted(self.OBSTACLES)],
            "bounds": {"width": self.WIDTH, "height": self.HEIGHT},
            "steps": self.steps,
            "score": self.score,
            "done": self.done,
            "available_actions": list(self.DELTAS),
            "board": self.render(),
        }

    def move(self, direction):
        if direction not in self.DELTAS:
            raise ValueError("direction 必须是 up、down、left 或 right")
        if self.done:
            result = self.observe()
            result.update({"ok": False, "retryable": False, "error": "episode_finished",
                           "message": "本局已经完成，请先重置游戏。"})
            return result

        dx, dy = self.DELTAS[direction]
        candidate = (self.ball[0] + dx, self.ball[1] + dy)
        self.steps += 1
        blocked = not (0 <= candidate[0] < self.WIDTH and 0 <= candidate[1] < self.HEIGHT)
        blocked = blocked or candidate in self.OBSTACLES
        if blocked:
            self.score -= 2
        else:
            self.ball = candidate
            self.score -= 1
        if self.ball == self.GOAL:
            self.done = True
            self.score += 100

        result = self.observe()
        result.update({
            "action": direction,
            "blocked": blocked,
            "message": "到达终点，游戏胜利！" if self.done else ("移动被边界或障碍挡住。" if blocked else "移动成功。"),
        })
        return result

    def render(self):
        rows = []
        for y in range(self.HEIGHT):
            row = []
            for x in range(self.WIDTH):
                pos = (x, y)
                row.append("●" if pos == self.ball else "◎" if pos == self.GOAL else "■" if pos in self.OBSTACLES else "·")
            rows.append(" ".join(row))
        return "\n".join(rows)
