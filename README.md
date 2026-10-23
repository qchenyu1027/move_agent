# Move Agent

Move Agent 是一个轻量的 LLM 工具调用项目，支持两种相互隔离的运行模式：

- `virtual`：在实时网页中控制虚拟网格小球，用于验证 LLM 的观察、规划和工具调用。
- `robot`：通过远程 HTTP API 控制真实 Luna 机器人。

项目要求 Python 3.10+。运行代码仅依赖标准库，自动测试使用 `pytest`。

以下命令默认都在项目根目录 `/home/yang/Desktop/move_agent` 中执行。

## 快速开始

首次创建环境：

```bash
cd /home/yang/Desktop/move_agent
conda env create -f environment.yml
conda activate move_agent
```

环境已存在时更新：

```bash
conda env update -f environment.yml --prune
```

启动虚拟游戏：

```bash
./scripts/start_virtual.sh
```

启动后访问 `http://127.0.0.1:8080`。局域网设备使用 `http://<运行机器IP>:8080`。

## LLM 配置与连接测试

默认配置为：

```text
LLM_BASE_URL=http://10.121.40.31:8000/v1
LLM_MODEL=Qwen3.5-9B
```

需要修改时设置环境变量：

```bash
export LLM_BASE_URL=http://服务器IP:8000/v1
export LLM_MODEL=模型名称
```

如果模型服务要求鉴权，可隐藏输入 API Key：

```bash
read -s -p "LLM_API_KEY: " LLM_API_KEY
echo
export LLM_API_KEY
```

单独测试 LLM 连接，不启动网页、机器人或 Agent Harness：

```bash
python3 scripts/check_llm.py
```

脚本依次检查 `/models` 和 `/chat/completions`。两项成功时退出码为 `0`，网络、HTTP 或响应格式异常时退出码为 `1`。超时时间可用 `LLM_CHECK_TIMEOUT_S` 调整。

程序不会自动读取 `.env` 或 `.env.example`，需要在启动前导出环境变量。

## 虚拟游戏模式

启动：

```bash
./scripts/start_virtual.sh
```

网页支持 LLM 自动控制、方向键或 WASD 手动控制、实时状态和动作日志，以及重置游戏。

示例任务：

```text
观察游戏，把小球移动到终点
```

## 真实机器人模式

### 1. 检查机器人 FastAPI

默认机器人地址是 `http://10.121.40.94:8000`。先执行只读健康检查：

```bash
./scripts/check_robot.sh
```

也可直接请求接口：

```bash
curl -4 --noproxy '*' -sS \
  --connect-timeout 10 \
  --max-time 15 \
  http://10.121.40.94:8000/health | python3 -m json.tool
```

提交动作前应确认响应包含：

```json
{
  "ok": true,
  "ready_for_motion": true,
  "api": {
    "busy": false
  }
}
```

### 2. 配置并启动

```bash
export ROBOT_API_BASE=http://10.121.40.94:8000
export ROBOT_HEALTH_TIMEOUT_S=15
export ROBOT_ACTION_TIMEOUT_S=600

read -s -p "ROBOT_API_KEY: " ROBOT_API_KEY
echo
export ROBOT_API_KEY

./scripts/start_robot.sh
```

启动脚本会检查 `ROBOT_API_KEY`、自动把机器人地址加入 `NO_PROXY`，然后启动网页；不会保存或显示密钥。

建议先在网页中执行只读任务：

```text
检查机器人健康状态，不要执行任何动作
```

确认现场安全后再执行运动任务，例如：

```text
移动到预设位姿 3
```

AprilTag 抓放必须明确提供标签和对应的标定文件：

```text
抓放 AprilTag 5，使用 tag5_left_tool_offset.yaml
```

### 真机工具和调用流程

真机模式向 LLM 提供以下工具：

- `robot_health`：读取健康、连接和忙闲状态。
- `move_robot`：移动到预设位姿，当前仅允许编号 `1`、`2`、`3`。
- `grasp_apriltag`：执行完整 AprilTag 抓放任务。
- `cancel_robot`：请求取消当前动作。

动作流程为：LLM 选择工具 → 检查 `/health` → 确认 `ready_for_motion=true` → 调用 `/move` 或 `/grasp` → 等待并返回结果。

只有 HTTP 200 且响应中的 `ok=true` 才表示动作成功。动作超时或连接断开时，执行结果可能未知，禁止自动重试。

## 启动参数

启动脚本是推荐方式。需要自定义时，也可以直接运行入口：

```bash
python3 app.py --mode virtual --port 8080
python3 app.py --mode robot --port 8080
python3 app.py --help
```

常用参数：

```text
--mode virtual|robot   运行模式
--host 0.0.0.0         网页监听地址
--port 8080            网页端口
--max-steps 40         单次 Agent 最大循环轮数
--cli                  使用终端界面
```

启动脚本也支持环境变量覆盖：

```bash
MOVE_AGENT_HOST=127.0.0.1 \
MOVE_AGENT_PORT=9000 \
MOVE_AGENT_MAX_STEPS=20 \
./scripts/start_virtual.sh
```

端口被占用时可设置其他端口，例如 `MOVE_AGENT_PORT=9000`。

终端模式和单次任务：

```bash
python3 app.py --mode virtual --cli
python3 app.py --mode robot --cli
python3 app.py --mode virtual "把小球向左移动一步"
```

## 测试

```bash
python3 -m pytest -q
```

只测试真实机器人 HTTP 客户端：

```bash
python3 -m pytest -q tests/test_robot_client.py
```

测试使用模拟 HTTP 响应，不会触发真实机器人动作。若不使用 Conda，可通过 `python3 -m pip install -r requirements-dev.txt` 安装测试依赖。

`robot_server/`（机器人端服务）的测试是独立的 unittest，需在该目录下运行，见 [robot_server/README.md](robot_server/README.md)；根目录的 `pytest.ini` 已把 `pytest` 限定为 `tests/`，不会收集它。

## 目录结构

```text
move_agent/
├── app.py                 # 主入口
├── web.py                 # 实时网页服务
├── config/                # LLM 和运行配置
├── game/                  # 虚拟游戏环境
├── harness/               # LLM 与工具调用循环
├── robot/                 # 虚拟与真机客户端
├── tools/                 # 提供给 LLM 的工具
├── robot_server/          # 机器人端 FastAPI 服务与 move/grasp 工具（需在该目录下运行）
├── web_static/            # 网页前端
├── tests/                 # 自动测试
├── scripts/               # 启动与连接检查脚本
├── environment.yml        # Conda 环境
├── requirements-dev.txt   # 测试依赖
├── prompt.md              # 虚拟模式提示词
└── prompt_robot.md        # 真机模式提示词
```

## 安全注意事项

- 不要把 `ROBOT_API_KEY` 写入源码、README、日志、聊天或 Git。
- 真机网页默认监听 `0.0.0.0`，只能在可信局域网内使用，不应暴露到公网。
- 运动前确认机器人工作范围内无人，并确保现场急停可用。
- `/cancel` 返回 HTTP 202 只代表取消请求已提交；必须继续确认 `api.busy=false`。
- 无线 DHCP 地址可能变化，应通过 `ROBOT_API_BASE` 配置机器人地址。

## 停止服务

在启动服务的终端按 `Ctrl+C`。

关闭网页服务不等于取消正在执行的机器人动作。机器人运动中应优先使用网页中的“紧急取消当前动作”，并等待健康状态显示 `busy=false`。
