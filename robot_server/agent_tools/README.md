# move / grasp 任务调度与 HTTP 服务

调度器位于 `agent_tools/scheduler.py`；工具位于 `tools/move_tool.py` 和
`tools/grasp_tool.py`。工具调用的控制脚本及其数据统一放在 `tools/scripts/`，
不依赖项目根目录下的其他文件。使用 Python 标准库，无新增依赖。

| 目录 | 内容 |
| --- | --- |
| `agent_tools/` | 调度器、HTTP 服务、启动脚本、任务示例，以及全部离线测试 |
| `tools/` | move / grasp 工具封装、参数校验与控制进程管理 |
| `tools/scripts/` | 只放动作运行所需文件：底盘移动 `base_move_world.py`、`base_move_world_pose_list.py`；AprilTag 抓取 `apriltag_left_tool_grasp_test.py`（自包含单文件：已合并 Tag 识别与目标位姿、左手 QP 控制两部分，不再依赖同目录其他脚本）；抓取数据 `tag3/tag5_left_tool_offset.yaml`、`left_tool_joint_snapshot_20261010.yaml` |

`tools/scripts/` 中是从项目根目录复制、并把路径改为相对自身目录的独立副本；根目录原文件保持不变，
工具不再调用它们。两边之后各自修改不会自动同步：用根目录的 `measure_tag_to_left_tool_offset.py`
重新标定后，需要把新的 yaml 复制到 `tools/scripts/`，或在项目根目录标定时加
`--output tools/scripts/tag5_left_tool_offset.yaml` 直接写入。

## 运行

在项目根目录预览任务（不加载 ROS、不访问机器人）：

```bash
python3 -B -m agent_tools.scheduler agent_tools/tasks.example.json
```

实际执行前加载原项目 ROS 环境，然后显式添加 `--execute`：

```bash
source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh
python3 -B -m agent_tools.scheduler agent_tools/tasks.example.json --execute
```

也支持 `python3 /home/liuqian/copy/test/agent_tools/scheduler.py <任务文件绝对路径>`。
控制脚本统一以 `tools/scripts/` 为工作目录运行；`options` 中的相对文件路径
（如 `offset-config`）以该目录为基准。任务文件路径相对于启动调度器时的工作目录。

## 任务格式

任务文件是非空 JSON 数组，按出现顺序执行。示例文件中的动作顺序需要按现场任务修改。

| 任务 | 必填目标 | 行为 |
| --- | --- | --- |
| `move` | `pose_ids: [1, 2, 3]` | 使用 `base_move_world_pose_list.py` 的默认位姿，按列表顺序移动 |
| `move` | `pose: [x, y, z, qx, qy, qz, qw]` | 使用 `base_move_world.py` 的世界位姿入口；z 仅检查 |
| `move` | `fixed: true` | 使用 `base_move_world.py` 内置的固定目标位姿 |
| `grasp` | 可选 `tag_id`，默认 5 | 执行 AprilTag 抓放；`grasp_and_lift: true` 时抓取并抬升后结束 |

move 的 `fixed`、`pose_ids` 和 `pose` 必须三选一。两类任务都支持 `options` 对象，
键为完整命令行选项名（省略 `--`），值为字符串或数值。例如：

```json
[
  {"tool": "move", "pose_ids": [1], "options": {"speed-m-s": 0.06}},
  {"tool": "grasp", "tag_id": 5, "options": {"offset-config": "tag5_left_tool_offset.yaml"}},
  {"tool": "move", "pose": [0.04681, 0.22712, 0.00603, 0, 0, 0.025117, 0.999685]}
]
```

只抓取并抬升 50 mm：

```json
[
  {"tool": "grasp", "tag_id": 5, "grasp_and_lift": true,
   "options": {"offset-config": "tag5_left_tool_offset.yaml"}}
]
```

move 支持原脚本的速度、容差、超时、定位等待、重试和 `vlahost-url` 参数；
默认位姿列表另支持 `pause-s`。grasp 支持标定文件、偏移配置、抓取、夹爪、
稳定窗口和底盘回位等参数。完整支持列表见工具文件中的 `MOVE_OPTIONS` / `GRASP_OPTIONS`。
调度器管理执行开关，任务中不能传入 `execute`、`help`、`check-only` 等改变任务语义的选项。

直接调用 HTTP `/grasp` 时默认使用 `"grasp_and_lift": true`：它会识别 Tag、接近、闭合夹爪、
保持夹爪闭合并抬升 50 mm，然后成功结束；不会移动底盘、松爪、返回快照位姿或调用 go_home。
只有在 `/tasks` 中显式写入 `"grasp_and_lift": false`，才执行完整的抓放、底盘移动、松爪、
回位和 Home 流程。
修改 `tag_id` 时需要同时选择对应的 `offset-config`；不会自动切换标定偏移。

## Python 调用与失败处理

```python
from agent_tools.scheduler import TaskScheduler

tasks = [{"tool": "move", "pose_ids": [1]}, {"tool": "grasp", "tag_id": 5}]
scheduler = TaskScheduler()
scheduler.run(tasks)                # 仅预览
# scheduler.run(tasks, execute=True)  # 真实执行
```

也可直接调用 `MoveTool().run(task)` 或 `GraspTool().run(task)`，这两个方法会真实执行。
调度前校验整个计划的结构、目标格式和选项名；参数范围及 ROS/定位就绪状态仍由原脚本在每项启动时校验。
预览成功不表示实机条件满足。控制日志直接输出到终端。

退出码 0 表示计划成功或预览成功；非零表示失败，后续任务不执行，不自动重试整个任务。
Ctrl-C 时通知当前控制脚本中断，等待其停止和清理后退出 130。

离线验证（在项目根目录执行）：

```bash
python3 -B -m unittest -v agent_tools.test_agent_tools agent_tools.test_base_move_world
```

## 局域网 HTTP 服务

服务位于 `agent_tools/api.py`，提供 `GET /health`、`POST /move`、
`POST /grasp`、`POST /tasks` 和 `POST /cancel`。接口直接执行现有调度器，
HTTP 响应要等机器人动作完成。
同一服务进程一次只执行一个动作；忙碌时返回 HTTP 409。不要启动多个服务进程或
Uvicorn worker，否则进程之间不能共用动作锁。

在机器人电脑的同一 Python 环境中安装所需依赖（已安装时可跳过），加载 ROS 环境，
设置自定义密钥后启动：

```bash
source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh
python3 -m pip install -r agent_tools/requirements-api.txt
read -rsp 'ROBOT_API_KEY: ' ROBOT_API_KEY; echo
export ROBOT_API_KEY
python3 -B -m agent_tools.api --host 0.0.0.0 --port 8000
```

部署机也提供 `agent_tools/start_api.sh`，它读取权限为 600 的
`/home/liuqian/.config/robot-agent-tools/api.env`，并固定使用单 worker：

```bash
nohup setsid agent_tools/start_api.sh >agent_tools/api.log 2>&1 < /dev/null &
```

`0.0.0.0` 表示监听机器人电脑的网络接口；调用时使用机器人电脑实际局域网 IP。
不设置 `ROBOT_API_KEY` 时服务拒绝启动。请只在可信局域网或有 TLS 的代理/VPN 下
提供接口，因为原生 HTTP 不加密请求头中的密钥。`/health` 不要求密钥，动作接口必须
传 `X-API-Key`。

在另一台电脑上，以机器人 IP 替换 `ROBOT_IP`，并在本机设置相同的密钥：

```bash
read -rsp 'ROBOT_API_KEY: ' ROBOT_API_KEY; echo
export ROBOT_API_KEY
curl http://ROBOT_IP:8000/health
curl -X POST -H "X-API-Key: $ROBOT_API_KEY" http://ROBOT_IP:8000/move
curl -X POST -H "X-API-Key: $ROBOT_API_KEY" -H 'Content-Type: application/json' \
  -d '{"pose_ids":[1]}' http://ROBOT_IP:8000/move
curl -X POST -H "X-API-Key: $ROBOT_API_KEY" -H 'Content-Type: application/json' \
  -d '{"tag_id":5,"options":{"offset-config":"tag5_left_tool_offset.yaml"}}' \
  http://ROBOT_IP:8000/grasp
curl -X POST -H "X-API-Key: $ROBOT_API_KEY" -H 'Content-Type: application/json' \
  -d '[{"tool":"move","pose_ids":[1]},{"tool":"grasp","tag_id":5}]' \
  http://ROBOT_IP:8000/tasks
curl -X POST -H "X-API-Key: $ROBOT_API_KEY" http://ROBOT_IP:8000/cancel
```

`POST /move` 不带请求体时执行 `base_move_world.py` 的内置固定目标；也可提供
`{"pose_ids":[1]}` 或 `{"pose":[x,y,z,qx,qy,qz,qw]}`。`POST /grasp` 不带请求体
时使用默认 `tag_id=5`，并只抓取后抬升 50 mm。若明确需要完整抓放流程，必须传
`{"grasp_and_lift":false}`。两者均可传 `options`，字段与上文任务格式相同。
`POST /tasks` 接收完整的非空任务数组，执行前校验整个计划。

`GET /health` 汇总 VLAHost HTTP/ROS 反馈、世界位姿新鲜度、关键 ROS
节点/服务/话题、底盘模式、夹爪和干预状态。`ready_for_motion` 为 false 时，
远程 Agent 不应提交动作；动作接口也会执行相同的前置检查并返回 HTTP 503，
不会启动控制脚本。

成功返回 `{"ok":true,"message":"..."}`；控制脚本失败返回 HTTP 500、
`ok:false` 和 `exit_code`；参数错误返回 HTTP 422；密钥错误返回 HTTP 401。
`POST /cancel` 返回 202 后向当前控制脚本发送 SIGINT，并保持动作锁直到脚本完成
零速与资源清理。HTTP 客户端断开或请求任务被取消时也触发相同路径；客户端超时后
仍应先检查 `/health`，不要直接重发动作。离线验证：

```bash
python3 -m pip install -r agent_tools/requirements-test.txt
python3 -B -m unittest -v agent_tools.test_agent_tools_api agent_tools.test_agent_tools agent_tools.test_base_move_world
```
