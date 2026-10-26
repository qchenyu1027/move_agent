# 远程 LLM 调用 Robot Agent Tools FastAPI

本文供远程 LLM 服务或其工具执行器调用 Luna 机器人。接口会触发真实底盘、机械臂和夹爪动作；调用端必须串行执行，并把 HTTP 结果当作唯一完成信号。

## 连接与鉴权

```text
Base URL: http://6.6.7.101:8000
健康检查: GET /health（无需鉴权）
动作接口: POST /move、/grasp、/tasks、/cancel
鉴权请求头: X-API-Key: <ROBOT_API_KEY>
Content-Type: application/json
```

由部署管理员通过安全配置把 `ROBOT_API_KEY` 注入 LLM 服务进程；不要把密钥写进提示词、日志、代码仓库或聊天内容。原生 HTTP 不加密，只能在可信局域网、VPN 或受控代理内使用。如果调用环境配置了 HTTP 代理，应把 `6.6.7.101` 加入 `NO_PROXY`。

```bash
export ROBOT_API_BASE=http://6.6.7.101:8000
export NO_PROXY="${NO_PROXY:+$NO_PROXY,}6.6.7.101"
```

## LLM 必须遵守的调用顺序

1. 每次动作前调用 `GET /health`。
2. 只有 HTTP 200 且 `ready_for_motion` 为 `true` 时才提交动作。
3. 一次只提交一个动作请求，并等待它返回；不要并发调用动作接口。
4. HTTP 200 且响应中 `ok=true` 才表示动作完成。
5. 超时、连接断开或结果未知时，禁止自动重发。先查 `/health`；若 `api.busy=true`，等待或调用 `/cancel`。
6. 动作失败后不要自行换参数重试。向上层返回原始 HTTP 状态、`message`、`errors`、`warnings` 和 `exit_code`。
7. 人员进入运动范围、现场要求停止或任务目标不明确时，调用 `/cancel`，随后轮询 `/health`，直到 `api.busy=false`。

`/health` 是只读接口。重点字段：

```json
{
  "ok": true,
  "ready_for_motion": true,
  "api": {"busy": false},
  "vlahost": {"ok": true, "ros_feedback": true, "connected": true},
  "errors": [],
  "warnings": []
}
```

不要只判断 `ok`；动作准入条件是 `ready_for_motion=true`。

## 动作接口

### 移动到预设位姿

预设编号目前只有 `1`、`2`、`3`。列表按顺序串行执行：

```http
POST /move
X-API-Key: <key>
Content-Type: application/json

{"pose_ids":[3,1,2]}
```

可选参数放在 `options` 中，名称不带 `--`：

```json
{
  "pose_ids": [3],
  "options": {
    "speed-m-s": 0.06,
    "position-tolerance-m": 0.07
  }
}
```

### 移动到显式世界位姿

`pose` 顺序固定为 `[x, y, z, qx, qy, qz, qw]`：

```json
{
  "pose": [0.04681, 0.22712, 0.00603, 0, 0, 0.025117, 0.999685]
}
```

`move` 必须且只能包含 `pose_ids`、`pose` 或 `fixed:true` 中的一种。LLM 不应使用无请求体的 `/move`，因为它会执行程序内置的固定目标，语义不够明确。

移动可选参数：

```text
vlahost-url, speed-m-s, yaw-speed-rad-s, yaw-command-hz,
yaw-command-duration-s, position-tolerance-m, yaw-tolerance-rad,
move-timeout-s, yaw-timeout-s, http-timeout-s,
pose-ready-timeout-s, pose-stable-s, odometry-retries
```

`pose_ids` 还支持 `pause-s`。

### AprilTag 抓取并抬起

```http
POST /grasp
X-API-Key: <key>
Content-Type: application/json

{"tag_id":5,"options":{"offset-config":"tag5_left_tool_offset.yaml"}}
```

`/grasp` 默认等价于 `grasp_and_lift=true`：成功响应表示 Tag 已抓住并抬升 50 mm，
夹爪保持闭合；不会移动底盘、松爪、返回快照位姿或调用 Home。修改 `tag_id` 时必须由任务配置
同时给出匹配的 `offset-config`；LLM 不得猜测标定文件。

只有明确需要完整抓放流程时才传：

```json
{"tag_id":5,"grasp_and_lift":false,"options":{"offset-config":"tag5_left_tool_offset.yaml"}}
```

### 串行任务计划

`/tasks` 接收非空 JSON 数组，先校验全部任务，再依次执行；任一任务失败即停止后续任务：

```json
[
  {"tool":"move","pose_ids":[3],"options":{"speed-m-s":0.06}},
  {"tool":"grasp","tag_id":5,"options":{"offset-config":"tag5_left_tool_offset.yaml"}},
  {"tool":"move","pose_ids":[1]}
]
```

请求地址为 `POST /tasks`，同样需要 `X-API-Key`。只有需要原子化的串行计划时才使用它；需要观察每一步结果时，应分别调用并在每一步前重新检查健康状态。

### 取消当前动作

```http
POST /cancel
X-API-Key: <key>
```

HTTP 202 表示已提出取消请求，不表示机器人已经停止清理。服务会向当前控制脚本发送 SIGINT，并保持动作锁，直到零速和资源清理路径结束。继续轮询 `/health`，确认 `api.busy=false` 后才能发下一项动作。没有任务时返回 HTTP 409。

## HTTP 状态处理

| 状态 | 含义 | LLM 行为 |
| --- | --- | --- |
| 200 | 请求完成 | 仍需检查 `ok=true` |
| 202 | 已接受取消请求 | 等待 `api.busy=false` |
| 401 | API Key 缺失或错误 | 停止，报告配置问题 |
| 409 | 正在执行其他任务、任务已取消或当前无任务可取消 | 不自动重发，先查健康状态 |
| 422 | JSON 结构、字段、目标或选项无效 | 修正调用参数；不得让 LLM猜机器人标定值 |
| 500 | 控制脚本启动或执行失败 | 停止后续任务，报告 `exit_code` |
| 503 | 机器人前置健康检查未通过 | 不会启动动作；返回并报告 `health` |

## 最小调用示例

```bash
curl --noproxy '*' -fsS "$ROBOT_API_BASE/health"

curl --noproxy '*' -sS -X POST \
  -H "X-API-Key: $ROBOT_API_KEY" \
  -H 'Content-Type: application/json' \
  -d '{"pose_ids":[3]}' \
  "$ROBOT_API_BASE/move"

curl --noproxy '*' -sS -X POST \
  -H "X-API-Key: $ROBOT_API_KEY" \
  "$ROBOT_API_BASE/cancel"
```

Python 调用端可以直接使用已有的 HTTP 客户端。动作请求可能持续数十秒到数分钟；客户端超时必须按“结果未知”处理，禁止自动重试：

```python
import os
import httpx

base_url = os.environ.get("ROBOT_API_BASE", "http://6.6.7.101:8000")
headers = {"X-API-Key": os.environ["ROBOT_API_KEY"]}

with httpx.Client(base_url=base_url, headers=headers, trust_env=False) as client:
    health = client.get("/health", timeout=5).json()
    if not health.get("ready_for_motion"):
        raise RuntimeError(f"robot not ready: {health}")
    response = client.post("/move", json={"pose_ids": [3]}, timeout=None)
    response.raise_for_status()
    result = response.json()
    if not result.get("ok"):
        raise RuntimeError(result)
```

## 服务端运维信息

服务运行在机器人电脑 `/home/liuqian/copy/test`，启动入口为 `agent_tools/start_api.sh`，日志为 `agent_tools/api.log`。密钥配置位于 `/home/liuqian/.config/robot-agent-tools/api.env`，权限应为 600。LLM 调用端不应读取或修改这些服务端文件。
