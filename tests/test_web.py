import json
import threading
import urllib.request
from http.server import ThreadingHTTPServer

from web import make_handler


class FakeRuntime:
    def snapshot(self):
        return {
            "mode": "robot",
            "robot": {"ok": True, "ready_for_motion": True},
            "agent": {"running": False},
            "events": [],
        }


def test_state_endpoint():
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(FakeRuntime()))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with urllib.request.urlopen(
            f"http://127.0.0.1:{server.server_port}/api/state"
        ) as response:
            data = json.load(response)
        assert data["mode"] == "robot"
        assert data["robot"]["ready_for_motion"] is True
    finally:
        server.shutdown()
        server.server_close()
