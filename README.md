# Move Agent

Move Agent 是一个轻量的 LLM 工具调用项目，支持两种相互隔离的运行模式：

- `virtual`：在网页中控制虚拟网格小球，用于验证 Agent 的观察、规划和工具调用。
- `robot`：通过远程 HTTP API 控制真实 Luna 机器人。

项目使用 Python 3.10+，运行代码仅依赖标准库。测试使用 `pytest`。

## 快速开始

首次配置环境：

```bash
cd /home/yang/Desktop/move_agent
conda env create -f environment.yml
conda activate move_agent
```

启动虚拟模式：

```bash
./scripts/start_virtual.sh
```

启动真机模式：

```bash
read -s -p "ROBOT_API_KEY: " ROBOT_API_KEY
echo
export ROBOT_API_KEY
./scripts/start_robot.sh
```

## 目录结构

项目已经扁平化，入口文件位于项目根目录：

```text
move_agent/
├── app.py                 # 主入口
├── web.py                 # 实时网页服务
├── config/                # LLM 和运行配置
├── game/                  # 虚拟游戏环境
├── harness/               # LLM 与工具调用循环
├── robot/
│   ├── client.py          # 虚拟环境客户端
│   └── http_client.py     # 真实机器人 HTTP 客户端
├── tools/
│   ├── robot_tools.py     # 真机 LLM 工具
│   └── ...                # 虚拟游戏工具
├── web_static/            # 网页前端
├── tests/                 # 自动测试
├── scripts/
│   ├── start_virtual.sh   # 启动虚拟网页
│   ├── start_robot.sh     # 启动真机网页
│   ├── check_robot.sh     # 只读机器人健康检查
│   └── check_llm.py       # 独立 LLM 连接测试
├── environment.yml        # Conda 环境
├── requirements-dev.txt   # 测试依赖
├── prompt.md              # 虚拟模式提示词
└── prompt_robot.md        # 真机模式提示词
```

## 1. 配置 LLM

默认配置：

```text
LLM_BASE_URL=http://10.121.40.31:8000/v1
LLM_MODEL=Qwen3.5-9B
```

可以在启动前覆盖：

```bash
export LLM_BASE_URL=http://10.121.40.31:8000/v1
export LLM_MODEL=Qwen3.5-9B
export LLM_API_KEY='仅在模型服务要求认证时设置'
```

程序不会自动读取 `.env` 或 `.env.example`。

## 环境安装

### 推荐：Conda

```bash
cd /home/yang/Desktop/move_agent
conda env create -f environment.yml
conda activate move_agent
```

环境已经存在时更新：

```bash
conda env update -f environment.yml --prune
```

### 使用现有 Python 环境

运行程序本身不需要安装第三方依赖。只有执行测试时需要安装：

```bash
python3 -m pip install -r requirements-dev.txt
```

检查版本：

```bash
python3 --version
python3 -m pytest --version
```

## 独立测试 LLM 连接

该测试不启动网页、不连接机器人，也不经过 Agent Harness：

```bash
cd /home/yang/Desktop/move_agent
python3 scripts/check_llm.py
```

默认检查：

```text
http://10.121.40.31:8000/v1
Qwen3.5-9B
```

使用其他地址或模型：

```bash
LLM_BASE_URL=http://服务器IP:8000/v1 \
LLM_MODEL=模型名称 \
LLM_CHECK_TIMEOUT_S=60 \
python3 scripts/check_llm.py
```

如果模型服务需要鉴权，先设置：

```bash
read -s -p "LLM_API_KEY: " LLM_API_KEY
echo
export LLM_API_KEY
python3 scripts/check_llm.py
```

脚本依次检查 `/models` 与 `/chat/completions`。两项均成功时退出码为 `0`；网络、HTTP 或响应格式异常时退出码为 `1`。

## 2. 运行虚拟游戏

```bash
cd /home/yang/Desktop/move_agent
python3 app.py --mode virtual --port 8080
```

也可以使用启动脚本：

```bash
./scripts/start_virtual.sh
```

本机浏览器访问：

```text
http://127.0.0.1:8080
```

局域网设备通过运行机器的 IP 访问，例如：

```text
http://192.168.12.139:8080
```

网页支持：

- LLM 自动控制小球
- 方向键或 WASD 手动控制
- 实时显示位置、步数、分数和动作日志
- 重置游戏

示例任务：

```text
观察游戏，把小球移动到终点
```

## 3. 运行真实机器人模式

机器人 FastAPI 默认地址：

```text
http://10.121.40.94:8000
```

### 3.1 检查网络和 FastAPI

健康检查不会触发机器人动作：

```bash
curl -4 --noproxy '*' -sS \
  --connect-timeout 10 \
  --max-time 15 \
  http://10.121.40.94:8000/health | python3 -m json.tool
```

也可以使用只读检查脚本：

```bash
./scripts/check_robot.sh
```

提交动作前应确认：

```json
{
  "ok": true,
  "ready_for_motion": true,
  "api": {
    "busy": false
  }
}
```

### 3.2 设置机器人环境变量

```bash
export ROBOT_API_BASE=http://10.121.40.94:8000
export NO_PROXY="${NO_PROXY:+$NO_PROXY,}10.121.40.94"
export ROBOT_HEALTH_TIMEOUT_S=15
export ROBOT_ACTION_TIMEOUT_S=600
```

使用隐藏输入设置 API Key，避免密钥进入命令历史：

```bash
read -s -p "ROBOT_API_KEY: " ROBOT_API_KEY
echo
export ROBOT_API_KEY
```

检查变量是否已设置，但不显示密钥：

```bash
test -n "$ROBOT_API_KEY" && echo "ROBOT_API_KEY 已设置" || echo "ROBOT_API_KEY 未设置"
```

### 3.3 启动真机网页

```bash
cd /home/yang/Desktop/move_agent
python3 app.py --mode robot --port 8080
```

或使用启动脚本：

```bash
./scripts/start_robot.sh
```

脚本会检查 `ROBOT_API_KEY` 是否存在、自动配置机器人地址的 `NO_PROXY`，然后启动真机网页；不会保存或显示密钥。

浏览器访问：

```text
http://127.0.0.1:8080
```

建议先执行只读任务：

```text
检查机器人健康状态，不要执行任何动作
```

确认现场安全后，才能提交真实运动任务，例如：

```text
移动到预设位姿 3
```

AprilTag 抓放必须明确提供标签和对应的标定文件：

```text
抓放 AprilTag 5，使用 tag5_left_tool_offset.yaml
```

## 4. 真机工具

真机模式向 LLM 提供以下工具：

- `robot_health`：读取健康、连接和忙闲状态。
- `move_robot`：移动到预设位姿，当前仅允许编号 `1`、`2`、`3`。
- `grasp_apriltag`：执行完整 AprilTag 抓放任务。
- `cancel_robot`：请求取消当前动作。

真实动作的调用流程：

```text
用户任务
  → LLM 选择工具
  → GET /health
  → 检查 ready_for_motion=true
  → POST /move 或 POST /grasp
  → 等待机器人返回最终结果
  → 将结果交回 LLM
```

只有 HTTP 200 且响应中的 `ok=true` 才表示动作成功。动作请求超时或连接断开时，结果可能未知，禁止自动重试。

## 5. 终端模式

虚拟环境：

```bash
python3 app.py --mode virtual --cli
```

真实机器人：

```bash
python3 app.py --mode robot --cli
```

也可以直接提交一次任务：

```bash
python3 app.py --mode virtual "把小球向左移动一步"
```

## 6. 其他启动参数

```bash
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

端口被占用时可更换端口：

```bash
python3 app.py --mode robot --port 9000
```

启动脚本可通过环境变量覆盖参数：

```bash
MOVE_AGENT_HOST=127.0.0.1 \
MOVE_AGENT_PORT=9000 \
MOVE_AGENT_MAX_STEPS=20 \
./scripts/start_virtual.sh
```

## 7. 测试

运行全部测试：

```bash
cd /home/yang/Desktop/move_agent
python3 -m pytest -q
```

只测试真实机器人 HTTP 客户端：

```bash
python3 -m pytest -q tests/test_robot_client.py
```

测试使用模拟 HTTP 响应，不会触发真实机器人动作。

## 8. 安全注意事项

- 不要把 `ROBOT_API_KEY` 写入源码、README、日志、聊天或 Git。
- `.env`、`runs/`、缓存文件已经加入 `.gitignore`。
- 真机网页默认监听 `0.0.0.0`，只能在可信局域网内使用，不应暴露到公网。
- 运动前确认机器人工作范围内无人，并确保现场急停可用。
- `/cancel` 返回 HTTP 202 只代表取消请求已提交；必须继续确认 `api.busy=false`。
- 无线 DHCP 地址可能变化，机器人地址应通过 `ROBOT_API_BASE` 配置，不要依赖硬编码地址。

## 9. 停止服务

停止网页服务：

```text
Ctrl+C
```

关闭网页服务不等同于取消正在执行的机器人动作。机器人运动中应优先使用网页中的“紧急取消当前动作”，并等待健康状态显示 `busy=false`。
