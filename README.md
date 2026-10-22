# Move Agent：虚拟小球游戏版

现同时支持隔离的虚拟游戏与真实 Luna 机器人模式。

## 真实机器人模式

先在当前终端注入配置（密钥不要写入仓库或聊天）：

```bash
export ROBOT_API_BASE=http://10.121.40.94:8000
export ROBOT_API_KEY='由机器人管理员提供的密钥'
export NO_PROXY="${NO_PROXY:+$NO_PROXY,}10.121.40.94"
python3 app.py --mode robot --port 8080
```

浏览器打开 `http://127.0.0.1:8080`。真机模式提供健康监控、LLM 任务入口、事件流和紧急取消。支持的模型工具为：健康检查、预设位姿 1/2/3 移动、带明确标定文件的 AprilTag 抓放，以及取消。

真机动作会先检查 `ready_for_motion=true`，串行执行且不自动重试。虚拟模式仍使用：

```bash
python3 app.py --mode virtual
```

轻量 LLM Harness，Python 3.10+，只使用标准库。连接 Qwen/OpenAI Chat Completions 兼容服务，让模型通过工具调用控制网格中的小球。

## 启动实时网页

```bash
cd /home/yang/Desktop/move_agent
python3 app.py
```

浏览器打开 `http://127.0.0.1:8080`。服务默认监听 `0.0.0.0`，局域网其他设备可通过 `http://运行机器IP:8080` 访问。

自定义端口：

```bash
python3 app.py --port 9000
```

网页支持实时显示模型每一步动作、分数、位置与事件流，也支持方向键/WASD手动控制。保留终端兼容模式：

```bash
python3 app.py --cli
```

默认模型地址为 `http://10.121.40.31:8000/v1`，模型为 `Qwen3.5-9B`。可通过 `LLM_BASE_URL`、`LLM_MODEL`、`LLM_API_KEY` 覆盖；`.env.example` 不会自动加载。

## 游戏规则

- `●`：由大模型控制的小球
- `■`：不可穿越的障碍
- `◎`：终点
- 坐标原点在左上角；小球每次移动一个格子
- 正常移动扣 1 分，撞击边界或障碍扣 2 分，到达终点加 100 分

大模型拥有三个工具：

- `observe_game`：观察棋盘、坐标、障碍、分数和完成状态
- `move_ball`：执行 `up`、`down`、`left`、`right`
- `reset_game`：在用户明确要求时开启新一局

每一步工具结果都会返回最新棋盘，模型必须基于真实返回状态继续规划。游戏对象在同一终端会话中持续存在，`runs/` 保存完整调用日志。

## 代码结构

- `game/environment.py`：游戏状态、碰撞、奖励和文本渲染
- `tools/`：模型可见的观察、移动和重置工具
- `robot/client.py`：虚拟环境适配层；保留类名以兼容原 Harness
- `harness/runtime.py`：模型与工具的循环
- `harness/model.py`：Chat Completions 兼容接口
- `prompt.md`：小球智能体规则
- `tests/test_game.py`：环境单元测试

原来的机器人动作文件仍保留在 `tools/` 中，但已经从工具注册表移除，不会被模型调用。以后接回真机时可以另建运行模式，不需要破坏当前虚拟环境。

## 测试

```bash
python3 -m pytest -q tests/test_game.py
```
