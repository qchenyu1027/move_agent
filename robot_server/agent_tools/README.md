# 导航 / 控制源切换 / 抓取 / 躯干 / 回 home / 手臂 / 夹爪任务调度与 HTTP 服务

调度器位于 `agent_tools/scheduler.py`；工具位于 `tools/move_tool.py`、
`tools/move_xy_tool.py`、`tools/grasp_tool.py`、`tools/torso_tool.py` 和
`tools/go_home_tool.py`、`tools/arm_teleop_tool.py` 和 `tools/gripper_tool.py`。工具调用的控制脚本及其数据统一放在 `tools/scripts/`，
`arm_teleop.py` 使用同目录的 `right_arm_teleop.py`，不依赖 robot_server 目录外的 Python 文件；全身 IK 使用 NumPy，依赖列在 `requirements-api.txt`。

| 目录 | 内容 |
| --- | --- |
| `agent_tools/` | 调度器、HTTP 服务、启动脚本、任务示例，以及全部离线测试 |
| `tools/` | 导航、抓取、躯干、手臂和夹爪工具封装、参数校验与控制进程管理 |
| `tools/scripts/` | VLAHost 世界位姿闭环 + Apex MoveRelative `native_nav_test.py`、站点数据 `pos.yaml`；AprilTag 抓取脚本、ROS `go_home.py`、`arm_teleop.py`、`gripper_control.py` 及动作数据文件 |

`tools/scripts/` 中是从原目录复制、并把路径改为相对自身目录的独立副本；`arm_teleop.py` 是同目录遥操实现的启动入口。原文件保持不变，
其他工具不再调用它们。两边之后各自修改不会自动同步：用根目录的 `measure_tag_to_left_tool_offset.py`
重新标定后，需要把新的 yaml 复制到 `tools/scripts/`，或在项目根目录标定时加
`--output tools/scripts/tag5_left_tool_offset.yaml` 直接写入。

## 运行

在 `robot_server/` 目录预览任务（不加载 ROS、不访问机器人）：

```bash
python3 -B -m agent_tools.scheduler agent_tools/tasks.example.json
```

实际执行前加载原项目 ROS 环境，然后显式添加 `--execute`：

```bash
source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh
python3 -B -m agent_tools.scheduler agent_tools/tasks.example.json --execute
```

也支持 `python3 /path/to/move_agent/robot_server/agent_tools/scheduler.py <任务文件绝对路径>`。
控制脚本统一以 `tools/scripts/` 为工作目录运行；`options` 中的相对文件路径
（如 `offset-config`）以该目录为基准。任务文件路径相对于启动调度器时的工作目录。

## 任务格式

任务文件是非空 JSON 数组，按出现顺序执行。示例文件中的动作顺序需要按现场任务修改。

| 任务 | 必填目标 | 行为 |
| --- | --- | --- |
| `move` | `pose_ids: [1, 2, 3, 4]` | 使用 pos.yaml 固定站点，按列表顺序发送一次 body-frame MoveRelative 并复核到站 |
| `move` | `pose: [x, y, z, qx, qy, qz, qw]` | 转换为世界 xyz/yaw 目标，读取 VLAHost 位姿后发送一次 body-frame 相对移动 |
| `move` | `fixed: true` | 使用 `native_nav_test.py` 的固定目标位姿 |
| `move_xy` | `x_m`、`y_m` | 相对当前 `camera_init` 世界坐标平移 x/y 米，保持当前高度和朝向 |
| `control_sources` | `ik_source`、`joint_source` | 依次调用 ROS `joint_cmd_mux/select` 和 `ik_request_mux/select`，确认活动源编号后继续下一任务 |
| `grasp` | 可选 `tag_id`，默认 5 | 执行 AprilTag 抓放；`grasp_and_lift: true` 时抓取并抬升后结束 |
| `head_object_grasp` | `side: left/right`；可选 `grasp_offset_mm: [x,y,z]` | 使用头部视觉 `/object_pose` 抓取物体；偏移单位为毫米，`[0,0,-40]` 表示抓取点下降 40 mm |
| `torso_home_down` | `offset_mm`（非零有限值，负数下降、正数上升） | 先到独立保存的原始双手水平 home，再只用躯干 IK 调整躯干高度；双臂关节保持 home |
| `go_home` | 无 | 通过 ROS `Trigger` 服务调用 `<namespace>/control/go_home`，并等待服务返回的计划运动时长后继续下一任务；默认命名空间为 `APEX_ROS_NAMESPACE` 或 `tj` |
| `arm_teleop` | `side: left` 或 `right` | `control-backend=auto` 时检测 `control/replay/ik_request` 是否有订阅者；有则走 ROS replay，没有则通过 VLAHost HTTP `/action` 控制末端位姿；默认启动键盘遥操，指定 `move-x-mm`、`move-y-mm` 或 `move-z-mm` 时执行一次性位移并自动退出 |
| `gripper` | `side`、`action` | 默认通过 ROS `std_msgs/Float32` 控制；`options.control-backend=vlahost` 时通过 VLAHost HTTP `/action` 的 `gripper_left/right` 控制；`action` 为 `open`、`close` 或 `set`，`position` 为目标开合度 0..1，`options.speed` 为开合速度（开合度/秒） |

完整搬运示例见 `agent_tools/bottle_transport_2_to_3.json`。该任务依次到站点 2、下蹲、沿世界 Y 轴移动 -0.2 m、切换 IK 源 2 和关节命令源 0、到达水瓶抓取点、单独设置左夹爪 0.4、回 home、左手上移 50 mm、到站点 3、左手下移 50 mm、设置夹爪 0.0。`head_object_grasp` 的 `reach-only` 选项在到达抓取点后结束，不执行脚本内的自动夹紧和抬升；`position-only` 只表示保持当前工具姿态。

`pose4` 的世界位置为 `(-0.4597338939, -0.0571276842, 0.0162584066)` m，
四元数换算得到目标 yaw 约为 `-3.074537` rad。可在加载 ROS 环境后运行
`/usr/bin/python3 tools/scripts/native_nav_test.py --pose-ids "[4]" --wait --execute`。
`move`、`move_xy` 调用 `/move/MoveRelativeCmd`：每个目标从 VLAHost `/base_world_pose` 读取 `camera_init -> body` 实时位姿，
按当前 yaw 将世界坐标差转换为 body-frame x/y，发送一个相对移动 goal，完成后再次读取 VLAHost 位姿复核 XY、Z 和 yaw。
导航不调用 `/server/CreateTaskByPose`，也不做 ROS 全局路径规划；每个站点最多两次尝试，只有可恢复失败才重试。
这条接口不做地图路径规划或绕障，运行前须确认行驶区域和转弯空间安全。

脚本会检查 VLAHost 位姿新鲜度和 `/move/MoveRelativeCmd` 是否可用；它不使用 `/server/CreateTaskByPose`，也不以导航节点的全局路径状态作为目标接口。确认现场路线畅通、急停可用后再执行导航。如果只是驻车模式导致不能移动，可显式允许程序先解除驻车：
`/usr/bin/python3 tools/scripts/native_nav_test.py --pose-ids "[4]" --wait --execute --release-parking`。
该选项调用 `/move/SetParkingMode`（`enable=false`），并等待 `/move/State.is_parking_mode=false` 后才发送目标；不带该选项不会自动解除驻车。任务 JSON 中可写 `"options": {"release-parking": true}`。动作返回成功后仍检查定位变化。

move 的 `fixed`、`pose_ids` 和 `pose` 必须三选一。两类任务都支持 `options` 对象，
键为完整命令行选项名（省略 `--`），值为字符串或数值。例如：

```json
[
  {"tool": "move", "pose_ids": [1], "options": {"position-tolerance-m": 0.05}},
  {"tool": "move_xy", "x_m": 0.20, "y_m": -0.10},
  {"tool": "grasp", "tag_id": 5, "options": {"offset-config": "tag5_left_tool_offset.yaml"}},
  {"tool": "move", "pose": [0.04681, 0.22712, 0.00603, 0, 0, 0.025117, 0.999685]},
  {"tool": "torso_home_down", "offset_mm": -10},
  {"tool": "go_home"}
]
```

`go_home` 只使用 ROS 服务。需要指定命名空间或超时时间时可写：
`{"tool": "go_home", "options": {"namespace": "tj", "wait-timeout-s": 30}}`。

`arm_teleop` 一次只控制一侧手臂，默认 `control-backend=auto`：启动时检查
`<namespace>/control/replay/ik_request` 的订阅者数量；检测到订阅者使用 ROS replay，数量为 0
则先请求 VLAHost `GET /health`，随后用 `POST /action` 的 `eef_left/eef_right` 字段持续发送目标。
VLAHost 接口要求左右位姿同时提供，程序会保持另一侧手臂当前 TF 位姿不变。也可以在 options 中显式写
`{"control-backend": "replay"}` 或 `{"control-backend": "vlahost", "vlahost-url": "http://6.6.7.100:8011"}`。
任务启动后需要在运行终端中按键：
`W/S` 前后、方向键左右/上下、`x/y/z` 绕固定轴旋转、`E` 接管、`Esc` 释放、`q` 退出。
例如：`{"tool": "arm_teleop", "side": "left", "options": {"incremental": true}}`。
一次性向上移动 50 mm：`{"tool": "arm_teleop", "side": "left", "options": {"move-z-mm": 50, "once-timeout-s": 15}}`。
调度器的 `--execute` 会自动传给遥操脚本；指定 `move-*-mm` 时不启动交互终端。

夹爪示例：

```json
[
  {"tool": "gripper", "side": "left", "action": "open", "position": 0.7,
   "options": {"speed": 0.4}},
  {"tool": "gripper", "side": "right", "action": "close",
   "options": {"speed": 0.8}}
]
```

可直接使用 `agent_tools/gripper_task_test.json` 预览或执行左夹爪测试。
VLAHost 测试可使用 `agent_tools/gripper_vlahost_task_test.json`。

夹爪命令话题默认是 `<namespace>/control/gripperValueL/R`，反馈话题默认是
`<namespace>/info/gripper_feedback_L/R`。程序会读取反馈的 `position[0]` 作为起点，按
`options.speed` 平滑发布到目标；也可以用 `options.start-position` 指定起点。
VLAHost 模式会从 `GET /state` 读取 `gripper_left/right.position` 作为起点，也可以显式指定
`options.start-position`；地址通过 `options.vlahost-url` 设置。

动作内先到独立保存的原始双手水平 home，再进行躯干高度调整：

```json
[
  {"tool": "torso_home_down", "offset_mm": -10},
  {"tool": "torso_home_down", "offset_mm": 10}
]
```

`offset_mm` 是相对原始 home 的有符号高度偏移：负数下降，正数上升，必须为非零有限数值。
默认不设人为幅度上限，可达范围由躯干 IK 收敛和 6 个 `Leg_J` 关节限位约束——够不到的偏移会在
IK 阶段报错且不启动动作。IK 采用连续化/热启动小步逼近，避免大偏移时初值发散。
当前 MJCF + 原始 home 下，保持 X 前后位置时下降约 `-234mm` 会触及 `Leg_J4=0.8 rad`；放开 X 后离线可继续到约 `-350mm`，但躯干会前移约 `0.45m`，继续下降会逐步触及其它关节限位，实际极限需在仿真和实机保护条件下确认。上升约 `+20mm` 受 `Leg_J3` 限位约束。如需人为上限，可传 `options: {"max-offset-mm": 50}`（默认为 inf 即不限制）。
工具不会修改当前 MJCF 的 home；原始姿态记录在 `tools/scripts/luna_horizontal_home.json`。
高度阶段只调整 6 个 `Leg_J` 躯干关节，双臂 14 个关节保持原始 home 角度；因此双手会随躯干一起移动，不再保持末端在世界坐标中的位置。IK 保持躯干左右方向（Y）、目标高度（Z）和躯干姿态，放开前后方向（X）约束，继续下降时躯干可能明显向前或向后移动，所以 `offset_mm` 只表示 Z 高度变化，不能理解为整机只做竖直平移。动作开始时仍会先让躯干和双臂回到原始 home。运行时默认每秒输出一次 user 输入、mux 输出、SDK 命令和实时反馈；可用 `--joint-log-hz` 调整频率。
头部不参与原始 home 到位判定或 IK 高度调整，但原始 home 阶段会发送
`luna_horizontal_home.json` 中的头部目标角度。当前文件中的 `Head_J1_joint=-0.2617994`
约为向下 15°；头部目标不影响躯干 IK。
执行后 mux 保持在 user 源。

只抓取并抬升 50 mm：

```json
[
  {"tool": "grasp", "tag_id": 5, "grasp_and_lift": true,
   "options": {"offset-config": "tag5_left_tool_offset.yaml"}}
]
```

头部视觉物体抓取示例：

```json
[
  {"tool": "head_object_grasp", "side": "left",
   "grasp_offset_mm": [0, 0, -40],
   "options": {"position-only": true, "ik-topic": "control/ik_request/vla",
                "ik-source": 2, "joint-source": 0}}
]
```

接近阶段会先等待目标稳定；默认位置误差不超过 50 mm、姿态误差不超过 5° 后进入下降阶段。
下降抓取和抬升仍使用 `position-tolerance-mm`、`angle-tolerance-deg` 的精确阈值。
如需调整接近判定，可在 `options` 中设置 `approach-position-tolerance-mm` 和
`approach-angle-tolerance-deg`。

视觉发布器在物体移动超过阈值时会向 `/object_pose_changed` 发布 `std_msgs/Bool`。
`head_object_grasp` 默认订阅该话题；在采集、预检、接近或下降阶段收到 `true` 后，
会先保持当前关节/末端位置，丢弃旧目标并重新采集稳定位姿，再按新位置规划。
闭合夹爪和抬升阶段忽略该通知。可通过 `options.object-pose-change-topic` 修改话题，
传空字符串可禁用该功能。

已提供同样参数的任务文件 `agent_tools/head_object_grasp_task.json`，可先预览，再实际执行：

```bash
python3 -B -m agent_tools.scheduler agent_tools/head_object_grasp_task.json
source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh
python3 -B -m agent_tools.scheduler agent_tools/head_object_grasp_task.json --execute
```

`move_xy` 的 `x_m`、`y_m` 是相对当前世界坐标的米数，正负号分别表示对应世界坐标轴方向；
也兼容不带单位后缀的 `x`、`y`，同一轴不能同时传两种字段。它从 VLAHost 读取当前位姿，将世界偏移转换为 body-frame 后发送一个 MoveRelative goal，并保持当前 yaw。`MoveRelativeCmd` 不支持速度参数；旧任务中的 `speed-m-s` 会明确提示不生效。grasp 支持标定文件、偏移配置、抓取、夹爪、
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

也可直接调用 `MoveTool().run(task)`、`MoveXYTool().run(task)` 或 `GraspTool().run(task)`，这些方法会真实执行。
调度前校验整个计划的结构、目标格式和选项名；参数范围及 ROS/定位就绪状态仍由原脚本在每项启动时校验。
预览成功不表示实机条件满足。控制日志直接输出到终端。

退出码 0 表示计划成功或预览成功；非零表示失败，后续任务不执行，不自动重试整个任务。
Ctrl-C 时通知当前控制脚本中断，等待其停止和清理后退出 130。

离线验证（在 `robot_server/` 目录执行）：

```bash
python3 -B -m unittest -v agent_tools.test_agent_tools agent_tools.test_agent_tools_api
```

## 局域网 HTTP 服务

服务位于 `agent_tools/api.py`，提供 `GET /health`、`POST /move`、`POST /move_xy`、
`POST /grasp`、`POST /head_object_grasp`、`POST /torso_home_down`、`POST /tasks` 和 `POST /cancel`。接口直接执行现有调度器，
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
  -d '{"x_m":0.20,"y_m":-0.10}' http://ROBOT_IP:8000/move_xy
curl -X POST -H "X-API-Key: $ROBOT_API_KEY" -H 'Content-Type: application/json' \
  -d '{"pose_ids":[1]}' http://ROBOT_IP:8000/move
curl -X POST -H "X-API-Key: $ROBOT_API_KEY" -H 'Content-Type: application/json' \
  -d '{"tag_id":5,"options":{"offset-config":"tag5_left_tool_offset.yaml"}}' \
  http://ROBOT_IP:8000/grasp
curl -X POST -H "X-API-Key: $ROBOT_API_KEY" -H 'Content-Type: application/json' \
  -d '{"side":"left","grasp_offset_mm":[0,0,-40],"options":{"position-only":true,"ik-topic":"control/ik_request/vla","ik-source":2,"joint-source":0}}' \
  http://ROBOT_IP:8000/head_object_grasp
curl -X POST -H "X-API-Key: $ROBOT_API_KEY" -H 'Content-Type: application/json' \
  -d '[{"tool":"move","pose_ids":[1]},{"tool":"grasp","tag_id":5}]' \
  http://ROBOT_IP:8000/tasks
curl -X POST -H "X-API-Key: $ROBOT_API_KEY" http://ROBOT_IP:8000/cancel
```

`POST /move` 不带请求体时执行 `native_nav_test.py` 的内置固定目标；也可提供
`{"pose_ids":[1]}` 或 `{"pose":[x,y,z,qx,qy,qz,qw]}`。`POST /grasp` 不带请求体
时使用默认 `tag_id=5`，并只抓取后抬升 50 mm。若明确需要完整抓放流程，必须传
`{"grasp_and_lift":false}`。两者均可传 `options`，字段与上文任务格式相同。
`POST /move_xy` 需要 `{"x_m":...,"y_m":...}`，表示相对当前世界坐标的 x/y 米数；
它保持当前 yaw 和 z，只执行平移。
`POST /tasks` 接收完整的非空任务数组，执行前校验整个计划。

`GET /health` 汇总 VLAHost HTTP/ROS 反馈、世界位姿新鲜度、关键 ROS
节点/服务/话题、底盘模式、夹爪和干预状态。`ready_for_motion` 为 false 时，
远程 Agent 不应提交动作；纯 `move`/`move_xy` 任务单独检查原生导航 ROS 接口，
包含抓取或躯干的任务仍检查完整健康状态。检查不通过会返回 HTTP 503，
不会启动控制脚本。

成功返回 `{"ok":true,"message":"..."}`；控制脚本失败返回 HTTP 500、
`ok:false` 和 `exit_code`；参数错误返回 HTTP 422；密钥错误返回 HTTP 401。
`POST /cancel` 返回 202 后向当前控制脚本发送 SIGINT，并保持动作锁直到脚本完成
零速与资源清理。HTTP 客户端断开或请求任务被取消时也触发相同路径；客户端超时后
仍应先检查 `/health`，不要直接重发动作。离线验证：

```bash
python3 -m pip install -r agent_tools/requirements-test.txt
python3 -B -m unittest -v agent_tools.test_agent_tools_api agent_tools.test_agent_tools
```
