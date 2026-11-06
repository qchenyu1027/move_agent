# robot_server：机器人控制任务与 FastAPI 接口

`robot_server` 是部署在机器人电脑上的控制服务。它把底盘导航、AprilTag 抓取、头部相机物体抓取、躯干升降、回 home、手臂移动和夹爪控制封装成任务，并通过 FastAPI 提供局域网 HTTP 接口。

目录结构：

```text
robot_server/
├── agent_tools/
│   ├── api.py                 # FastAPI 服务
│   ├── scheduler.py           # JSON 任务串行调度器
│   ├── start_api.sh           # 机器人端启动脚本
│   └── *.json                 # 任务示例
└── tools/
    ├── *_tool.py              # 参数校验和命令构建
    └── scripts/               # 实际 ROS/VLAHost 控制脚本
```

所有命令都应在 `robot_server` 目录执行。这里的 `agent_tools` 和 `tools` 是相互导入的 Python 包；仓库根目录也有一个同名 `tools` 包，在错误目录启动可能导致导入错误。

## 1. 环境准备

机器人电脑需要已经启动 ROS 2、Apex 控制节点、VLAHost，并能看到所需的 ROS topic/service/action。推荐使用系统 Python，避免用户目录中的 NumPy 2.x 覆盖 ROS 二进制包依赖的 NumPy 1.x：

```bash
cd /home/liuqian/copy/test/move_agent/robot_server
source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh
export PYTHONNOUSERSITE=1
/usr/bin/python3 -m pip install -r agent_tools/requirements-api.txt
```

不要把真实 API key 写入仓库。服务启动前必须设置 `ROBOT_API_KEY`。

## 2. 先预览任务，再执行

调度器默认只构建并打印命令，不会连接或移动机器人：

```bash
cd /home/liuqian/copy/test/move_agent/robot_server
/usr/bin/python3 -B -m agent_tools.scheduler agent_tools/tasks.example.json
```

确认现场状态、急停和运动范围后，加载 ROS 环境并显式添加 `--execute`：

```bash
cd /home/liuqian/copy/test/move_agent/robot_server
source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh
export PYTHONNOUSERSITE=1
/usr/bin/python3 -B -m agent_tools.scheduler \
  agent_tools/tasks.example.json --execute
```

任务按 JSON 数组顺序串行执行。某个任务退出码非零时，后续任务停止；Ctrl-C 会向当前控制脚本发送 SIGINT，并等待其完成清理。

## 3. 可用 task

任务文件必须是非空 JSON 数组。每个对象的 `tool` 只能使用下面这些值：

| `tool` | 作用 | 主要字段 |
|---|---|---|
| `move` | 移动底盘到固定站点或显式世界位姿 | `fixed`、`pose_ids`、`pose` 三选一；可选 `options` |
| `move_xy` | 按当前世界坐标的 x/y 偏移移动底盘 | `x_m`、`y_m`（也支持 `x`、`y`）；可选 `options` |
| `control_sources` | 切换 IK 源和关节命令源 | `ik_source`、`joint_source` |
| `grasp` | AprilTag 左手抓取流程 | 可选 `tag_id`、`grasp_and_lift`、`options` |
| `head_object_grasp` | 根据头部相机发布的 `PoseStamped` 抓取物体 | `side`；可选 `grasp_offset_mm`、`object_pose_topic`、`pose_request_topic`、`options` |
| `torso_home_down` | 回原始双臂 home 后调整躯干高度 | `offset_mm`（负数下降、正数上升）；可选 `options` |
| `go_home` | 调用 ROS `control/go_home` 服务 | 可选 `options` |
| `arm_teleop` | 手臂交互遥操或一次性位移 | `side`；可选 `options` |
| `gripper` | 控制左/右夹爪开合 | `side`、`action`；`action` 为 `open`、`close` 或 `set` |

`options` 的键使用完整命令行选项名，但不写前导 `--`，例如 `"position-only": true`。调度器会校验未知字段和数值范围。

### 3.1 底盘 `move`

`fixed`、`pose_ids`、`pose` 必须且只能指定一个：

```json
[
  {"tool": "move", "pose_ids": [2]},
  {"tool": "move", "pose": [0.45, 0.16, 0.0, 0, 0, 0.0, 1]},
  {"tool": "move", "fixed": true}
]
```

`pose_ids` 使用 `tools/scripts/pos.yaml` 中的站点数据。`pose` 为 `[x, y, z, qx, qy, qz, qw]` 世界位姿。导航脚本从 VLAHost `/base_world_pose` 获取 `camera_init -> body` 实时位姿，将世界目标转换为 Apex `MoveRelative` 所需的机体相对位移，并在到达后复核。

常用 `options`：`release-parking`、`move-timeout-s`、`position-tolerance-m`、`yaw-tolerance-rad`、`max-attempts`、`pause-s`、`vlahost-url`。

### 3.2 底盘 `move_xy`

`x_m`、`y_m` 是世界坐标偏移，单位为米，保持当前高度和朝向：

```json
[
  {"tool": "move_xy", "x_m": 0.20, "y_m": -0.10}
]
```

### 3.3 控制源 `control_sources`

先切换关节命令源，再切换 IK 源，并确认 ROS mux 返回目标源：

```json
[
  {"tool": "control_sources", "ik_source": 2, "joint_source": 0}
]
```

### 3.4 AprilTag 抓取 `grasp`

默认 `tag_id` 为 5。通过 `grasp_and_lift: true` 可只执行抓取并抬升；`false` 执行完整抓放流程：

```json
[
  {
    "tool": "grasp",
    "tag_id": 5,
    "grasp_and_lift": true,
    "options": {"offset-config": "tag5_left_tool_offset.yaml"}
  }
]
```

`offset-config` 的相对路径以 `tools/scripts/` 为基准。更换 Tag 时必须使用对应的标定偏移文件。

### 3.5 头部相机物体抓取 `head_object_grasp`

该任务等待头部视觉程序发布 `PoseStamped`，再通过 TF 将物体位置转换到 `base_link`。默认话题是 `/object_pose`，位姿请求话题是 `/f2g_pose/request`。`grasp_offset_mm` 单位为毫米，例如 `[0, 0, -40]` 表示在物体位置下方 40 mm 抓取：

```json
[
  {
    "tool": "head_object_grasp",
    "side": "left",
    "object_pose_topic": "/object_pose",
    "pose_request_topic": "/f2g_pose/request",
    "grasp_offset_mm": [0, 0, -40],
    "options": {
      "control-backend": "joint",
      "position-only": true,
      "reach-only": true,
      "ik-source": 2,
      "joint-source": 0,
      "model-xml": "/home/liuqian/copy/Apex_Deploy/robot_node/marvin_description/mjcf/Gento_Luna/Gento_Luna.xml"
    }
  }
]
```

常用选项：

- `position-only`：忽略物体姿态，保持工具当前姿态，只移动位置。
- `reach-only`：到达抓取点后结束，不闭合夹爪、不抬升。
- `approach-only`：到达预抓取点后结束。
- `control-backend=joint`：使用本地 Pinocchio IK 和 user joint source 2。
- `control-backend=ros`：发布 ROS IK 请求。
- `control-backend=http`：通过 VLAHost HTTP 控制。
- `approach-front-mm`、`approach-height-mm`：调整预抓取点。
- `intermediate-offset-mm`：先移动到相对物体原点的中间点。
- `approach-position-tolerance-mm`、`reach-position-tolerance-mm`：位置稳定阈值。

执行此任务前必须确认 RGB-D/F2G-Pose 发布器、相机 TF（例如 `base_link -> luna_head_calibrated_optical`）和末端 TF 均可用。

### 3.6 躯干 `torso_home_down`

负值下降，正值上升，必须是非零有限数值：

```json
[
  {"tool": "torso_home_down", "offset_mm": -100}
]
```

程序会先回到 `tools/scripts/luna_horizontal_home.json` 保存的 home，再用躯干 IK 调整高度；实际范围由 MJCF 关节限位决定。

### 3.7 `go_home`

调用 ROS `control/go_home` 服务并等待服务返回的计划时长：

```json
[
  {"tool": "go_home", "options": {"namespace": "tj", "wait-timeout-s": 30}}
]
```

### 3.8 手臂 `arm_teleop`

`side` 为 `left` 或 `right`。指定 `move-x-mm`、`move-y-mm`、`move-z-mm` 时执行一次性移动并退出；不指定时进入键盘遥操：

```json
[
  {
    "tool": "arm_teleop",
    "side": "left",
    "options": {
      "control-backend": "joint",
      "move-z-mm": 50,
      "once-command-duration-s": 2.0
    }
  }
]
```

一次性移动仍会发布真实控制命令，执行前必须确认机器人安全。可用选项还包括 `once-timeout-s`、`once-tolerance-mm`、`once-settle-s`、`max-joint-speed-rad-s`、`vlahost-control-space`。

### 3.9 夹爪 `gripper`

`position` 范围为 0 到 1：

```json
[
  {"tool": "gripper", "side": "left", "action": "open", "options": {"speed": 0.4}},
  {"tool": "gripper", "side": "left", "action": "set", "position": 0.5}
]
```

夹爪默认使用 ROS 话题；`options.control-backend` 可设置为 `vlahost`。常用选项：`speed`、`start-position`、`hold-s`、`namespace`、`vlahost-url`。

## 4. 启动 FastAPI 服务

### 4.1 直接启动

```bash
cd /home/liuqian/copy/test/move_agent/robot_server
source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh
export PYTHONNOUSERSITE=1
export ROBOT_API_KEY='请替换为随机长字符串'

/usr/bin/python3 -B -m agent_tools.api \
  --host 0.0.0.0 \
  --port 8000
```

服务只启动一个 Uvicorn worker。不要使用 `--workers > 1`，因为动作锁和取消状态只存在于单个进程内。

### 4.2 使用 `start_api.sh`

准备权限为 600 的配置文件 `/home/liuqian/.config/robot-agent-tools/api.env`：

```bash
mkdir -p /home/liuqian/.config/robot-agent-tools
cat > /home/liuqian/.config/robot-agent-tools/api.env <<'EOF'
ROBOT_API_KEY=请替换为随机长字符串
ROBOT_API_HOST=0.0.0.0
ROBOT_API_PORT=8000
ROBOT_PYTHON=/usr/bin/python3
EOF
chmod 600 /home/liuqian/.config/robot-agent-tools/api.env
```

启动：

```bash
cd /home/liuqian/copy/test/move_agent/robot_server
agent_tools/start_api.sh
```

后台运行：

```bash
nohup setsid agent_tools/start_api.sh \
  > agent_tools/api.log 2>&1 < /dev/null &
```

## 5. FastAPI 接口

假设机器人 IP 为 `ROBOT_IP`，端口为 `8000`。除 `GET /health` 外，所有接口都需要请求头 `X-API-Key`。

### 接口总览

| 方法 | 路径 | 鉴权 | 说明 |
|---|---|---|---|
| GET | `/health` | 否 | 检查 VLAHost、ROS 节点/服务/话题、位姿新鲜度和动作锁 |
| POST | `/move` | 是 | 执行一个 `move` task |
| POST | `/move_xy` | 是 | 执行一个 `move_xy` task |
| POST | `/torso_home_down` | 是 | 执行一个 `torso_home_down` task |
| POST | `/grasp` | 是 | 执行一个 `grasp` task，默认抓取并抬升 |
| POST | `/head_object_grasp` | 是 | 执行一个 `head_object_grasp` task |
| POST | `/tasks` | 是 | 执行完整 JSON task 数组 |
| POST | `/cancel` | 是 | 请求安全取消当前任务 |

### 5.1 健康检查

```bash
curl -s http://ROBOT_IP:8000/health | python3 -m json.tool
```

动作接口只会在健康检查通过且服务不忙时启动任务。`ready_for_motion: false` 时，先修复返回的 `errors`；服务不会启动机器人动作。

### 5.2 单动作调用

```bash
export ROBOT_API_KEY='与机器人服务相同的 key'
export ROBOT_URL='http://ROBOT_IP:8000'

# 到固定站点
curl -sS -X POST "$ROBOT_URL/move" \
  -H "X-API-Key: $ROBOT_API_KEY" \
  -H 'Content-Type: application/json' \
  -d '{"pose_ids":[2]}'

# 世界坐标相对平移 0.2 m、-0.1 m
curl -sS -X POST "$ROBOT_URL/move_xy" \
  -H "X-API-Key: $ROBOT_API_KEY" \
  -H 'Content-Type: application/json' \
  -d '{"x_m":0.20,"y_m":-0.10}'

# 躯干下降 100 mm
curl -sS -X POST "$ROBOT_URL/torso_home_down" \
  -H "X-API-Key: $ROBOT_API_KEY" \
  -H 'Content-Type: application/json' \
  -d '{"offset_mm":-100}'

# AprilTag 抓取并抬升
curl -sS -X POST "$ROBOT_URL/grasp" \
  -H "X-API-Key: $ROBOT_API_KEY" \
  -H 'Content-Type: application/json' \
  -d '{"tag_id":5,"grasp_and_lift":true,"options":{"offset-config":"tag5_left_tool_offset.yaml"}}'

# 头部相机物体抓取
curl -sS -X POST "$ROBOT_URL/head_object_grasp" \
  -H "X-API-Key: $ROBOT_API_KEY" \
  -H 'Content-Type: application/json' \
  -d '{"side":"left","grasp_offset_mm":[0,0,-40],"options":{"position-only":true,"reach-only":true,"control-backend":"joint","ik-source":2,"joint-source":0}}'
```

`POST /move` 空 body 时等价于 `{"fixed": true}`。`POST /grasp` 空 body 时使用 `tag_id=5`，并默认 `grasp_and_lift=true`，不会进入完整抓放流程。

### 5.3 串行任务调用

`POST /tasks` 的 body 直接是 JSON 数组，不要再包一层 `tasks` 字段：

```bash
curl -sS -X POST "$ROBOT_URL/tasks" \
  -H "X-API-Key: $ROBOT_API_KEY" \
  -H 'Content-Type: application/json' \
  --data-binary @agent_tools/bottle_transport_2_to_3.json
```

也可以直接发送：

```json
[
  {"tool": "move", "pose_ids": [2]},
  {"tool": "control_sources", "ik_source": 2, "joint_source": 0},
  {"tool": "head_object_grasp", "side": "left",
   "grasp_offset_mm": [0, 0, -40],
   "options": {"position-only": true, "reach-only": true}},
  {"tool": "gripper", "side": "left", "action": "set", "position": 0.5}
]
```

服务会先校验完整数组，再做健康检查，然后串行执行；任一项失败时停止后续任务。

### 5.4 取消任务

```bash
curl -sS -X POST "$ROBOT_URL/cancel" \
  -H "X-API-Key: $ROBOT_API_KEY"
```

返回 `202` 只表示已发出安全取消请求。当前控制脚本完成清理后，动作锁才会释放；不要立刻并发提交下一项动作。

## 6. Python 调用示例

```python
import httpx

url = "http://ROBOT_IP:8000"
headers = {"X-API-Key": "与机器人服务相同的 key"}

with httpx.Client(base_url=url, headers=headers, timeout=None) as client:
    health = client.get("/health")
    health.raise_for_status()
    if not health.json().get("ready_for_motion"):
        raise RuntimeError(health.json())

    response = client.post("/tasks", json=[
        {"tool": "move", "pose_ids": [2]},
        {"tool": "gripper", "side": "left", "action": "open"},
    ])
    print(response.status_code, response.json())
```

动作请求会一直等待到任务完成，因此客户端超时应设置为 `None` 或足够大的值。客户端断开连接时，服务会请求当前任务安全停止。

## 7. HTTP 状态码与故障排查

| 状态码 | 含义 |
|---|---|
| `200` | 请求成功，任务已完成 |
| `202` | `/cancel` 已接受取消请求 |
| `401` | 缺少或错误的 `X-API-Key` |
| `409` | 机器人正在执行其他任务，或没有可取消的任务 |
| `422` | JSON 结构、字段或数值非法 |
| `503` | 健康检查未通过，任务尚未启动 |
| `500` | 控制脚本执行失败；响应中包含退出码 |

常见问题：

1. **`ModuleNotFoundError: agent_tools` 或导入了错误的 `tools`**：进入 `robot_server/` 后再启动，不要从仓库根目录直接运行模块。
2. **Pinocchio 报 `_ARRAY_API not found`**：使用 `/usr/bin/python3`，加载 ROS 环境，并设置 `PYTHONNOUSERSITE=1`。
3. **`ready_for_motion: false`**：先查看 `/health` 返回的 `errors`，检查 VLAHost、ROS 节点、`/move/MoveRelativeCmd`、`/tj/joint_states` 和所需 TF。
4. **`409 机器人正在执行其他任务`**：服务是单动作锁，等待上一任务结束或调用 `/cancel`，不要启动第二个 Uvicorn worker。
5. **头部抓取失败**：确认 F2G-Pose 已启动、`/object_pose` 有新消息、`/f2g_pose/request` 可用，以及 `base_link` 到相机和末端的 TF 存在。

## 8. 测试与安全

离线测试不连接机器人：

```bash
cd /home/liuqian/copy/test/move_agent/robot_server
PYTHONNOUSERSITE=1 /usr/bin/python3 -B -m unittest -v \
  agent_tools.test_agent_tools \
  agent_tools.test_base_move_world
```

FastAPI 服务默认监听 `127.0.0.1`；需要局域网访问时才使用 `--host 0.0.0.0`。原生 HTTP 不加密 API key，建议只在可信局域网、VPN 或 TLS 反向代理后使用。执行任何带 `--execute` 的任务前，确认机器人周围无障碍物、急停可用，并先用调度器预览命令。

`tools/eef_delta_tool.py` 当前已同步到仓库，但 `agent_tools/scheduler.py` 尚未把 `eef_delta` 注册为可调度 task，因此不能通过当前 `/tasks` 直接调用；如需启用，应同时把它加入调度器工具注册表并补充对应控制脚本。
