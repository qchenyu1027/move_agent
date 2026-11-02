# 远程 LLM 调用 Robot Agent Tools FastAPI

本文供远程 LLM 服务或其工具执行器调用 Luna 机器人。接口会触发真实底盘、机械臂和夹爪动作；调用端必须串行执行，并把 HTTP 结果当作唯一完成信号。

## 连接与鉴权

```text
Base URL: http://6.6.7.101:8000
健康检查: GET /health（无需鉴权）
动作接口: POST /move、/move_xy、/grasp、/eef_delta、/tasks、/cancel
视觉接口: GET /vision/bottles（需要鉴权，只读）
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
  "ros": {
    "services": {"ok": true, "missing": []},
    "topics": {"ok": true, "missing": []}
  },
  "errors": [],
  "warnings": []
}
```

不要只判断 `ok`；动作准入条件是 `ready_for_motion=true`。

## 动作接口

### 移动到预设位姿

预设编号为 `1`、`2`、`3`、`4`。`4` 对应世界位置
`(-0.4597338939, -0.0571276842, 0.0162584066)` m、yaw 约 `-3.074537` rad。
列表按顺序串行执行：

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

输入四元数转换为目标 yaw。执行脚本从 VLAHost `/base_world_pose` 读取 `camera_init -> body` 当前世界位姿，
将目标的世界坐标差转换为 body-frame，并通过 `/move/MoveRelativeCmd` 只发送一个相对移动 goal，随后复查世界定位。
它不调用 `/server/CreateTaskByPose`，也不使用 ROS 全局路径规划。此接口不提供地图绕障。

`move` 必须且只能包含 `pose_ids`、`pose` 或 `fixed:true` 中的一种。LLM 不应使用无请求体的 `/move`，因为它会执行程序内置的固定目标，语义不够明确。

移动可选参数：

```text
vlahost-url, speed-m-s, release-parking, service-timeout,
http-timeout-s, pose-ready-timeout-s, pose-stable-s, move-timeout-s,
position-tolerance-m, yaw-tolerance-rad, yaw-reference-rad,
yaw-max-travel-rad, max-attempts
```

`pose_ids` 还支持 `pause-s`。

### 相对移动底盘 x/y

`POST /move_xy` 根据 VLAHost `/base_world_pose` 当前世界位姿，沿世界 x/y 轴计算目标；
`x_m`、`y_m` 可以为负数，目标保持当前 z 和 yaw。将世界坐标偏移转换为 body-frame 后发送一个 MoveRelative goal；这不是全局路径规划。
也兼容不带单位后缀的 `x`、`y`，同一轴不能同时传两种字段。

```http
POST /move_xy
X-API-Key: <key>
Content-Type: application/json

{"x_m":0.20,"y_m":-0.10}
```

可选参数与 `/move` 相同：

```json
{
  "x_m": 0.20,
  "y_m": -0.10,
  "options": {
    "position-tolerance-m": 0.05
  }
}
```

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

### EEF 短位移

`POST /eef_delta` 沿当前 `base_link` 坐标系对指定 EEF 做一次原子平移，单次位移
向量范数不超过 200 mm：

```http
POST /eef_delta
X-API-Key: <key>
Content-Type: application/json

{"side":"left","dx_mm":20,"dy_mm":0,"dz_mm":10}
```

`side` 必须是 `left` 或 `right`；三个位移字段都必须提供，且不能全部为零。
它不是键盘遥操，也不接受连续轨迹；机器人端不会调用 `arm_teleop.py`，而是按 RPent
逻辑校验 `/hil/snapshot`、本地 TF、VLAHost 路由和未选中 EEF 保持，并通过
`/action_chunk` 发送 10 帧绝对双臂 EEF 目标。调用前仍必须通过 `/health` 的运动准入检查。

### 串行任务计划

`/tasks` 接收非空 JSON 数组，先校验全部任务，再依次执行；任一任务失败即停止后续任务：

```json
[
  {"tool":"move","pose_ids":[3],"options":{"position-tolerance-m":0.05}},
  {"tool":"move_xy","x_m":0.20,"y_m":-0.10},
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

## 当前瓶子视觉快照

`GET /vision/bottles?prompt=water%20bottle` 通过机器人 API 代理到本机
F2G-Pose 服务（默认 `F2G_VISION_URL=http://127.0.0.1:7862`）。它只读取一
张同步 RGB-D 图像，不发布 ROS、不移动底盘。响应包含 `objects[]` 的
`id`、`position_m`、`quaternion_xyzw`、`rotation_matrix`、`size_m` 和
`bbox_xyxy`，以及 `image.base64` 的 JPEG 标注图。ID 是当前图像内 SAM 3
的从左到右序号，不是跨帧跟踪 ID。视觉请求超时可用
`F2G_VISION_TIMEOUT_S` 配置。

底盘视觉对齐接口同样通过 F2G 服务执行：

```http
GET /base_alignment/preview?instance_index=2&prompt=water%20bottle
X-API-Key: <key>
```

该接口会重新获取当前图像，选择指定瓶子 index，转换到 `base_link`，并返回
`relative_move`；不发送底盘动作。确认结果后再调用：

```http
POST /base_alignment/move
X-API-Key: <key>
Content-Type: application/json

{"instance_index": 2, "prompt": "water bottle"}
```

成功响应中的 `executed=true` 才表示 MoveRelative goal 已发送并完成。该原型
仍未加入额外的运动安全和新鲜度门控；运行前需由上层确认现场安全。

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
  -H 'Content-Type: application/json' \
  -d '{"x_m":0.20,"y_m":-0.10}' \
  "$ROBOT_API_BASE/move_xy"

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

服务在机器人电脑的 `move_agent/robot_server` 目录运行，启动入口为 `agent_tools/start_api.sh`，日志为 `agent_tools/api.log`。密钥配置位于 `/home/liuqian/.config/robot-agent-tools/api.env`，权限应为 600。LLM 调用端不应读取或修改这些服务端文件。
