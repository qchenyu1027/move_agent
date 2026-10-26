"""模型连接配置及运行参数。"""
import os

DEFAULT_BASE_URL = "http://10.121.40.31:8000/v1"
DEFAULT_MODEL = "Qwen3.5-9B"
RUNS_DIR = "runs"


def llm_settings():
    return {"base_url": os.getenv("LLM_BASE_URL", DEFAULT_BASE_URL).rstrip("/"),
            "model": os.getenv("LLM_MODEL", DEFAULT_MODEL), "api_key": os.getenv("LLM_API_KEY", "")}
