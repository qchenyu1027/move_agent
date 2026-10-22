#!/usr/bin/env python3
"""A tiny terminal agent for an OpenAI-compatible Qwen server.

The server can be started with swift deploy (host 0.0.0.0, port 8000).
This client intentionally uses only Python's standard library.
"""

from __future__ import annotations

import argparse
import base64
import json
import mimetypes
import os
import sys
import urllib.error
import urllib.request
from typing import Any


DEFAULT_BASE_URL = "http://192.168.10.101:8000/v1"
#DEFAULT_BASE_URL = "http://100.101.50.6:8000/v1"
DEFAULT_MODEL = "Qwen3.5-9B"


class QwenAgent:
    """Minimal chat agent that keeps conversation history on the client."""

    def __init__(
        self,
        base_url: str = DEFAULT_BASE_URL,
        model: str = DEFAULT_MODEL,
        system_prompt: str = "你是一个 helpful assistant，请用中文简洁回答。",
        timeout: int = 120,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.timeout = timeout
        self.messages: list[dict[str, Any]] = [
            {"role": "system", "content": system_prompt}
        ]

    @staticmethod
    def _image_data_url(image_path: str) -> str:
        """Convert a local image into the data URL accepted by OpenAI-compatible APIs."""
        content_type = mimetypes.guess_type(image_path)[0] or "application/octet-stream"
        with open(image_path, "rb") as image_file:
            encoded = base64.b64encode(image_file.read()).decode("ascii")
        return f"data:{content_type};base64,{encoded}"

    def chat(
        self,
        text: str,
        *,
        image_path: str | None = None,
        temperature: float = 0.7,
    ) -> str:
        if image_path:
            content: list[dict[str, Any]] = [
                {"type": "text", "text": text},
                {
                    "type": "image_url",
                    "image_url": {"url": self._image_data_url(image_path)},
                },
            ]
        else:
            content = text
        self.messages.append({"role": "user", "content": content})
        payload = {
            "model": self.model,
            "messages": self.messages,
            "temperature": temperature,
            "max_tokens": 512,
            "stream": False,
        }
        request = urllib.request.Request(
            f"{self.base_url}/chat/completions",
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                # Swift accepts requests without a key; this also works if one is configured.
                "Authorization": f"Bearer {os.getenv('QWEN_API_KEY', 'EMPTY')}",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                result: dict[str, Any] = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"Qwen API 返回 HTTP {exc.code}: {detail}") from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(
                f"无法连接 Qwen 服务 {self.base_url}：{exc.reason}"
            ) from exc

        try:
            answer = result["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError) as exc:
            raise RuntimeError(f"Qwen API 返回格式异常：{result}") from exc

        self.messages.append({"role": "assistant", "content": answer})
        return answer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="与局域网 Qwen 模型进行多轮对话")
    parser.add_argument(
        "--base-url",
        default=os.getenv("QWEN_BASE_URL", DEFAULT_BASE_URL),
        help="OpenAI 兼容接口地址（默认读取 QWEN_BASE_URL）",
    )
    parser.add_argument(
        "--model",
        default=os.getenv("QWEN_MODEL", DEFAULT_MODEL),
        help="模型名（默认读取 QWEN_MODEL）",
    )
    parser.add_argument("--temperature", type=float, default=0.7)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    agent = QwenAgent(args.base_url, args.model)
    print(f"已连接配置：{args.base_url}，模型：{args.model}")
    print("输入 exit 或 quit 退出，输入 clear 清空上下文。")
    print("发送图片：/image 图片路径 你的问题")
    while True:
        try:
            text = input("你：").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n再见！")
            return 0
        if not text:
            continue
        if text.lower() in {"exit", "quit"}:
            return 0
        if text.lower() == "clear":
            agent.messages = agent.messages[:1]
            print("已清空对话上下文。")
            continue
        try:
            image_path = None
            if text.startswith("/image "):
                command = text[len("/image ") :].strip()
                image_path, separator, text = command.partition(" ")
                if not separator or not text.strip():
                    print("用法：/image 图片路径 你的问题")
                    continue
                text = text.strip()
            print(
                f"Qwen：{agent.chat(text, image_path=image_path, temperature=args.temperature)}"
            )
        except RuntimeError as exc:
            print(f"错误：{exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
