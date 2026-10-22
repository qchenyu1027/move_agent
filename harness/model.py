"""Harness 内部的真实模型 HTTP 客户端。"""
import json
import urllib.error
import urllib.request
from config.settings import llm_settings
from tools import TOOL_SCHEMAS

class ChatClient:
    """Client for a tool-calling Chat Completions compatible HTTP endpoint."""

    def __init__(self, tools=None):
        settings = llm_settings()
        self.base_url = settings["base_url"]
        self.model = settings["model"]
        self.key = settings["api_key"]
        self.tools = TOOL_SCHEMAS if tools is None else tools
        if not self.base_url or not self.model:
            raise ValueError("请设置 LLM_BASE_URL 和 LLM_MODEL")
        # Direct connection: ignore proxy environment variables for this client only.
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def complete(self, messages):
        payload = {"model": self.model, "messages": messages, "tools": self.tools}
        headers = {"Content-Type": "application/json"}
        if self.key:
            headers["Authorization"] = "Bearer " + self.key
        request = urllib.request.Request(
            self.base_url + "/chat/completions",
            data=json.dumps(payload).encode(), headers=headers, method="POST",
        )
        try:
            with self.opener.open(request, timeout=60) as response:
                result = json.load(response)
        except urllib.error.HTTPError as exc:
            # Do not echo provider response bodies, which may contain sensitive input.
            raise RuntimeError(f"LLM HTTP错误 {exc.code}；检查地址、模型、认证和工具调用支持") from None
        except urllib.error.URLError:
            raise RuntimeError("LLM连接失败；检查服务地址和网络") from None
        message = result["choices"][0]["message"]
        return {k: message[k] for k in ("role", "content", "tool_calls") if k in message}
