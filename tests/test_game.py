from game import BallGame


def test_reset_and_observe():
    game = BallGame()
    state = game.observe()
    assert state["ball"] == {"x": 1, "y": 6}
    assert state["goal"] == {"x": 10, "y": 1}
    assert not state["done"]


def test_wall_collision_costs_score():
    game = BallGame()
    game.ball = (0, 0)
    result = game.move("up")
    assert result["blocked"] is True
    assert result["ball"] == {"x": 0, "y": 0}
    assert result["score"] == -2


def test_reachable_path_wins():
    game = BallGame()
    for direction in ["right"] * 6 + ["up"] * 6 + ["right"] * 3 + ["down"]:
        result = game.move(direction)
    assert result["done"] is True
    assert result["ball"] == {"x": 10, "y": 1}
    assert result["score"] > 0
