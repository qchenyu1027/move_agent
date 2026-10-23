#!/usr/bin/env python3
"""独立检查 OpenAI Chat Completions 兼容 LLM 服务。"""
import json
import os
import sys
import urllib.error
import urllib.request


BASE_URL = os.getenv("LLM_BASE_URL", "http://10.121.40.31:8000/v1").rstrip("/")
MODEL = os.getenv("LLM_MODEL", "Qwen3.5-9B")
API_KEY = os.getenv("LLM_API_KEY", "")
TIMEOUT = float(os.getenv("LLM_CHECK_TIMEOUT_S", "60"))


def request(method, path, payload=None):
    headers = {"Accept": "application/json"}
    if payload is not None:
        headers["Content-Type"] = "application/json"
    if API_KEY:
        headers["Authorization"] = "Bearer " + API_KEY
    req = urllib.request.Request(
        BASE_URL + path,
        data=None if payload is None else json.dumps(payload).encode("utf-8"),
        headers=headers,
        method=method,
    )
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(req, timeout=TIMEOUT) as response:
        return response.status, json.load(response)


def main():
    print(f"LLM Base URL: {BASE_URL}")
    print(f"LLM Model:    {MODEL}")
    try:
        status, models = request("GET", "/models")
        print(f"[1/2] /models: HTTP {status}")
        names = [item.get("id") for item in models.get("data", []) if isinstance(item, dict)]
        if names:
            print("      可用模型: " + ", ".join(str(name) for name in names))

        status, result = request("POST", "/chat/completions", {
            "model": MODEL,
            "messages": [{"role": "user", "content": "只回复：连接成功"}],
            "max_tokens": 32,
        })
        content = result["choices"][0]["message"].get("content", "")
        print(f"[2/2] /chat/completions: HTTP {status}")
        print("      模型回复: " + content.strip())
        print("LLM 连接测试成功。")
        return 0
    except urllib.error.HTTPError as exc:
        print(f"LLM HTTP 错误：{exc.code}", file=sys.stderr)
    except urllib.error.URLError as exc:
        print(f"LLM 连接失败：{exc.reason}", file=sys.stderr)
    except (TimeoutError, OSError) as exc:
        print(f"LLM 连接或超时错误：{exc}", file=sys.stderr)
    except (KeyError, ValueError, TypeError) as exc:
        print(f"LLM 响应格式错误：{exc}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
