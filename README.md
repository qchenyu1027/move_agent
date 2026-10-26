# Move Agent

Move Agent 使用大模型把用户自然语言路由到真实 Luna 机器人的移动或抓取函数。没有匹配动作时，大模型直接进行简短问答。

当前只向 LLM 提供两个动作：

- `move_robot(pose_id)`：移动到预设位置 1、2、3。
- `grasp_object(tag_id?)`：执行 AprilTag 抓放；默认使用标签 5，也支持标签 3。

底层速度、超时、标定文件和机器人地址不交给 LLM 决定。每个用户请求只调用一次 LLM；一次最多执行一个机器人动作，不自动重试。

## 工作方式

```text
用户网页输入
    ↓
LLM 判断用户意图
    ├── 移动请求 → move_robot
    ├── 抓取请求 → grasp_object
    └── 其他问题 → LLM 直接回答
                         ↓
              RealRobotClient 自动检查 /health
                         ↓
              机器人电脑 FastAPI Server
                         ↓
                   移动/抓取脚本
```

项目需要部署在两台电脑上：

- LLM 调用端：运行仓库根目录中的网页、Harness 和机器人 HTTP Client。
- 机器人电脑：运行 `robot_server/` 中的 FastAPI、ROS 和实际控制脚本。

## 自然语言示例

移动到位置 2：

```text
移动到2
```

LLM 会选择：

```json
{"name":"move_robot","arguments":{"pose_id":2}}
```

抓取默认物体：

```text
抓住这个东西
```

LLM 会选择：

```json
{"name":"grasp_object","arguments":{}}
```

“这个东西”固定表示已经标定的 AprilTag 5，程序自动使用 `tag5_left_tool_offset.yaml`。也可以明确指定另一个已标定目标：

```text
抓取 AprilTag 3
```

此时程序自动使用 `tag3_left_tool_offset.yaml`。抓取是完整的抓放、底盘移动和回位流程，并不只是闭合夹爪。

普通输入不会调用机器人，例如：

```text
你好，请介绍一下你能做什么
```

## 1. 配置机器人电脑

进入机器人端目录。所有机器人端命令都必须从该目录运行：

```bash
cd /home/yang/Desktop/move_agent/robot_server
```

安装 FastAPI 服务依赖；已有环境可以跳过：

```bash
python3 -m pip install -r agent_tools/requirements-api.txt
```

`agent_tools/start_api.sh` 会加载机器人 ROS 环境，并读取：

```text
/home/liuqian/.config/robot-agent-tools/api.env
```

配置文件至少包含：

```bash
ROBOT_API_KEY=请填写实际密钥
ROBOT_API_HOST=0.0.0.0
ROBOT_API_PORT=8000
```

限制配置文件权限：

```bash
chmod 600 /home/liuqian/.config/robot-agent-tools/api.env
```

前台启动 FastAPI：

```bash
agent_tools/start_api.sh
```

机器人 Server 提供以下接口：

- `GET /health`：只读健康检查，不需要 API Key。
- `POST /move`：执行移动，需要 API Key。
- `POST /grasp`：执行抓取，需要 API Key。
- `POST /cancel`：取消当前动作，需要 API Key。

详细说明见 [robot_server/README.md](robot_server/README.md)。

## 2. 配置 LLM 调用端

进入项目根目录并创建环境：

```bash
cd /home/yang/Desktop/move_agent
conda env create -f environment.yml
conda activate move_agent
```

环境已经存在时更新：

```bash
conda env update -f environment.yml --prune
```

配置 LLM。以下是当前默认值，需要时可以覆盖：

```bash
export LLM_BASE_URL=http://10.121.40.31:8000/v1
export LLM_MODEL=Qwen3.5-9B
```

如果 LLM 服务需要认证：

```bash
read -s -p "LLM_API_KEY: " LLM_API_KEY
echo
export LLM_API_KEY
```

配置机器人 Server 地址和相同的 API Key：

```bash
export ROBOT_API_BASE=http://10.121.40.94:8000
read -s -p "ROBOT_API_KEY: " ROBOT_API_KEY
echo
export ROBOT_API_KEY
```

程序不会自动读取 `.env`，这些变量必须在启动程序的同一个终端中设置。

## 3. 测试连接

只测试 LLM，不连接机器人：

```bash
python3 scripts/check_llm.py
```

只检查机器人 FastAPI，不触发运动：

```bash
./scripts/check_robot.sh
```

健康检查应包含：

```json
{
  "ok": true,
  "ready_for_motion": true,
  "api": {"busy": false}
}
```

如果 `ready_for_motion=false`，Client 不会提交动作。应先根据响应中的 `errors` 和 `warnings` 修复机器人端状态。

## 4. 启动网页

推荐使用启动脚本：

```bash
./scripts/start_robot.sh
```

本机浏览器打开：

```text
http://127.0.0.1:8080
```

局域网设备打开：

```text
http://<LLM调用端电脑IP>:8080
```

修改网页地址或端口：

```bash
MOVE_AGENT_HOST=127.0.0.1 \
MOVE_AGENT_PORT=9000 \
./scripts/start_robot.sh
```

也可以直接启动：

```bash
python3 app.py --port 8080
```

网页中的“紧急取消”按钮不经过 LLM，会直接请求机器人 `/cancel`。

## 终端和单次调用

交互式终端：

```bash
python3 app.py --cli
```

提交单条指令：

```bash
python3 app.py "移动到2"
```

## 配置项

LLM：

- `LLM_BASE_URL`：兼容 OpenAI Chat Completions 的服务地址。
- `LLM_MODEL`：模型名称，模型必须支持工具调用。
- `LLM_API_KEY`：可选的 LLM 密钥。
- `LLM_CHECK_TIMEOUT_S`：独立连接测试超时。

机器人 Client：

- `ROBOT_API_BASE`：机器人 FastAPI 地址。
- `ROBOT_API_KEY`：机器人动作接口密钥。
- `ROBOT_HEALTH_TIMEOUT_S`：健康检查超时，启动脚本默认 15 秒。
- `ROBOT_ACTION_TIMEOUT_S`：动作请求超时，启动脚本默认 600 秒。

网页：

- `MOVE_AGENT_HOST`：监听地址，默认 `0.0.0.0`。
- `MOVE_AGENT_PORT`：监听端口，默认 `8080`。

## 测试代码

调用端测试全部使用模拟机器人响应，不会触发真实运动：

```bash
python3 -m pytest -q
```

机器人端离线测试：

```bash
cd /home/yang/Desktop/move_agent/robot_server
python3 -m pip install -r agent_tools/requirements-test.txt
python3 -B -m unittest -v \
  agent_tools.test_agent_tools \
  agent_tools.test_agent_tools_api \
  agent_tools.test_base_move_world
```

## 主要代码位置

```text
move_agent/
├── app.py                         启动入口
├── web.py                         实时网页后端
├── web_static/                    网页前端
├── harness/model.py               LLM HTTP Client
├── harness/runtime.py             单次意图路由和工具执行
├── prompt_robot.md                LLM 行为规则
├── tools/robot_tools.py           提供给 LLM 的两个工具
├── robot/http_client.py           机器人 FastAPI Client
├── robot_server/agent_tools/api.py 机器人 FastAPI Server
├── robot_server/tools/            移动和抓取程序
├── scripts/                       启动与连接检查脚本
└── tests/                         不连接真机的调用端测试
```

## 常见问题

端口已被占用：

```text
OSError: [Errno 98] Address already in use
```

可以换一个端口：

```bash
python3 app.py --port 9000
```

机器人连接超时：

- 确认机器人电脑的无线 IP 是否变化。
- 在调用端执行 `./scripts/check_robot.sh`。
- 确认机器人 FastAPI 监听 `0.0.0.0:8000`。
- 检查两台电脑的路由和防火墙。

LLM 只回答但没有调用函数：

- 运行 `python3 scripts/check_llm.py`。
- 确认模型支持 Chat Completions `tools` 和 `tool_calls`。
- 查看 `runs/*.jsonl` 中记录的模型原始响应。

## 安全说明

- 启动前确认机器人运动范围内无人，并确保现场急停可用。
- 不要把 `ROBOT_API_KEY` 写入源码、Git、README、日志或聊天。
- 机器人动作接口使用原生 HTTP，建议只在可信局域网、VPN 或 TLS 代理后使用。
- Client 在每个动作前强制执行健康检查；健康检查失败时不会发送动作。
- 超时或断线表示动作结果未知，程序不会自动重试。
- `/cancel` 返回 HTTP 202 只表示取消请求已提交，需要继续确认 `busy=false`。
- 关闭网页服务不等于取消正在运行的机器人动作。
