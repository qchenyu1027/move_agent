# 水瓶抓取路径规划器

`safe_grasp_planner.py` 从 `/object_pose` 获取物体位姿，转换到 `base_link` 后生成并执行抓取航路。
本程序控制手臂和夹爪；`/object_pose` 需要由 F2G-Pose 等程序提供。航路为：

```text
当前工具 → 垂直抬高 → 平移到预抓取点上方 → 预抓取点闭合夹爪 → 从该点抬升
```

## 在 RViz 查看抓取点

先启动 F2G-Pose 位姿发布程序，并触发一次检测，使 `/object_pose` 收到当前水瓶位姿。
然后在一个终端运行：

```bash
cd /home/liuqian/copy/test/move_agent/robot_server
source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh
export PYTHONNOUSERSITE=1

/usr/bin/python3 -B -m path_planner.rviz_grasp_preview \
  --table-z 0.70 \
  --object-pose-topic /object_pose \
  --grasp-offset-mm 0 0 -40
```

在使用同一 ROS_DOMAIN_ID 的终端启动 `rviz2`。将 **Fixed Frame** 设为 `base_link`，
添加 **MarkerArray**，话题选 `/f2g_pose/grasp_markers`；再添加 **RobotModel**
和 **TF**，即可把标记与机器人叠加查看。紫色为视觉物体点，
黄色为实际闭合夹爪的 `pregrasp` 点，橙色线为关键航路，红色箭头从蓝色点指向黄色点。
旧版红色 `grasp` 点已从执行航路中移除。
如果抓取命令使用了不同的 `--grasp-offset-mm`、`--approach-front-mm` 等参数，
预览命令也要传入相同的值；例如将黄色闭合点沿 `base_link` 的 X 负方向移动
20 mm：`--grasp-offset-mm -20 0 -40`。

这个节点只发布 RViz 标记，不发布关节、IK、夹爪或导航命令。标记保留最后一帧有效
位姿 30 秒，超时自动清除；可用 `--display-seconds` 调整。标记表示规划器使用的
**工具目标点**，不表示夹爪已经接触水瓶。桌面高度、物体 TF 和工具 TF 必须有效。

水平平移的终点位于**预抓取点上方**，不再专门去水瓶正上方。规划器使用工具坐标判断
是否到位，不检测夹爪与水瓶的实际接触。
从当前工具到预抓取点上方逐点执行；之后直接到黄色 `pregrasp`，闭合夹爪，再从
黄色点垂直抬升。不再前进到旧红色 `grasp`，也不等待 `pregrasp[1/5]` 等中间点。
直达目标仍进行离散桌面高度检查和关节目标限速；
实际关节运动不保证严格沿检查过的直线，运行时也会监测工具的桌面安全高度。

## 坐标和方向

`--table-z` 以及预览用的物体、工具位置均使用 `base_link` 坐标系，单位为米。
以 `-mm` 结尾的参数使用毫米。`--grasp-offset-mm X Y Z` 直接沿 `base_link` 的
X、Y、Z 轴偏移，**不是沿夹爪朝向偏移**。实际相机坐标系下的 `/object_pose` 需要
有效 TF 才能转换为 `base_link`。

```text
参考点 = 物体位置 + grasp-offset-mm / 1000
闭合点 pregrasp = 参考点 + (-approach-front-mm, 0, approach-height-mm) / 1000
抓取前平移高度 = max(当前工具 Z, 参考点 Z + overhead-mm/1000,
                    table-z + table-clearance-mm/1000)
抓取后抬升点 = (闭合点 X, 闭合点 Y, 闭合点 Z + lift-height-mm/1000)
```

默认 `--grasp-offset-mm 0 0 -40` 表示参考点 X、Y 不变，Z **下降 40 mm**。
黄色闭合点在参考点的 **X 负方向 80 mm、Z 正方向 30 mm**。此时闭合点相对视觉物体点
是 `(-80, 0, -10) mm`。改变 `--grasp-offset-mm`、`--approach-front-mm` 或
`--approach-height-mm` 都会改变实际闭合位置；调整前先在 RViz 核对黄色点。

## 航路与桌面参数

| 参数 | 默认值 | 如何影响运动 |
| --- | --- | --- |
| `--table-z` | 必填 | 桌面在 `base_link` 中的 Z，单位 m；必须使用现场实测值。 |
| `--table-clearance-mm` | `100` | 除起始点外，航路点和离散线段离桌面的最低高度；黄色闭合点也必须满足此下限。增大可能使抬手更高。 |
| `--start-clearance-mm` | `20` | 起始工具点允许的最低桌面余量；不是抬手高度。第一段只允许垂直抬高。 |
| `--grasp-offset-mm X Y Z` | `0 0 -40` | 视觉物体点到参考点的偏移；也会等量移动黄色闭合点。 |
| `--approach-front-mm` | `80` | 黄色闭合点相对参考点沿 X 负方向的距离；增大会让实际闭合位置更靠后。 |
| `--approach-height-mm` | `30` | 黄色闭合点相对参考点高多少；增大会让实际闭合位置更高。 |
| `--overhead-mm` | `80` | 抓取前平移高度至少比参考点高多少；实际高度还受当前手位和桌面安全高度约束。 |
| `--lift-height-mm` | `150` | 夹爪闭合后，从黄色闭合点垂直抬升多少；不控制抓取前的抬高。 |

不要为了让过低的抓取点通过检查而随意降低桌面安全余量；先核对桌面 Z、物体位置和工具接触点。

## 执行、到位与控制参数

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `--max-step-mm` | `10` | 线段的安全检查最大离散间距；到预抓取点上方之前也是逐点执行步长。减小会增加平移阶段点数；增大会使安全检查采样更稀疏。 |
| `--settle-time` | `0.4` | 每个点进入容差后连续保持的秒数；例如改为 `0.05` 可缩短停顿，仍不是连续轨迹。 |
| `--position-tolerance-mm` | `10` | 工具到位及本地 IK 解的最大位置误差。 |
| `--angle-tolerance-deg` | `5` | 工具到位及本地 IK 解的最大姿态误差；即使位置误差很小，姿态超限也会停止。 |
| `--max-joint-speed-rad-s` | `0.3` | `joint` 后端每周期单关节目标的限速，单位 rad/s；不保证整条轨迹加速度连续。 |
| `--max-body-deviation-deg` | `5` | 躯干关节偏离接管时姿态的停止阈值。 |
| `--motion-timeout` | `30` | 每个航路点的到位超时，单位秒。 |
| `--close-position` | `0.7` | 黄色 `pregrasp` 到位后的夹爪位置命令，范围 `0..1`；不表示已验证抓住水瓶。 |
| `--side` | `left` | 使用 `left` 或 `right` 手臂。 |

当前版本在 `above_pregrasp` 之后直接执行 `pregrasp`，到位即闭合夹爪，随后抬升。
日志不再出现 `pregrasp[1/5]` 或 `grasp[8/9]`，也不会向旧红点运动。
如果肉眼看到夹爪已经接触水瓶但程序还要前进，应先停止并检查工具接触点和
`--grasp-offset-mm`；放宽 IK 容差不能修正抓取目标的位置。

## 输入和接口参数

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `--object-pose-topic` | `/object_pose` | 物体位姿输入话题；执行时需要有效相机 TF。 |
| `--object-pose-change-topic` | `/object_pose_changed` | 运动中收到物体位置变化通知会停止当前航路并重新采集。 |
| `--stable-samples` | `5` | 开始规划前要求的连续有效物体位姿样本数，至少为 2。 |
| `--stable-position-mm` | `5` | 这些样本允许的最大位置变化。 |
| `--capture-timeout` | `45` | 等待稳定物体位姿的超时，单位秒。 |
| `--control-backend` | `joint` | `joint` 使用本地 Pinocchio 单臂 IK 和 user joint 命令；`ros` 发布 ROS IK。 |
| `--joint-source` | `joint` 时 `2`；`ros` 时 `0` | 关节命令 mux 源编号。 |
| `--ik-source` | `2` | `ros` 后端的 IK mux 源编号。 |
| `--ik-topic` | `control/ik_request/vla` | `ros` 后端的 IK 请求话题，相对于 `--namespace`。 |
| `--namespace` | `/tj` | 控制和反馈话题的 ROS 命名空间。 |
| `--model-xml` | ROS 包中的 `Gento_Luna.xml` | 本地 IK 与工具安装位姿使用的模型。 |
| `--gripper-topic` | 按所选手臂自动选择 | 覆盖默认夹爪控制话题。 |
| `--execute` | 不启用 | 启用真实手臂和夹爪动作；不加时只预览航路。 |
| `--object-position X Y Z` | 不设置 | 离线预览的物体位置，单位 m，必须同时提供 `--current-position`。 |
| `--current-position X Y Z` | 不设置 | 离线预览的当前工具位置，单位 m。 |
| `--output PATH` | 不设置 | 离线预览时把关键航路点保存为 JSON。 |

## 离线预览

以下坐标只是示例，预览不会发布运动命令；先用现场数据核对航路点：

```bash
cd /home/liuqian/copy/test/move_agent/robot_server
export PYTHONNOUSERSITE=1

/usr/bin/python3 -B -m path_planner.safe_grasp_planner \
  --table-z 0.70 \
  --current-position 0.20 0.10 0.80 \
  --object-position 0.40 0.20 0.90 \
  --grasp-offset-mm -20 0 -40 \
  --output /tmp/grasp_plan.json
```

已启动 `/object_pose` 时，也可省略 `--execute`、`--object-position`、`--current-position`
来预览现场物体位姿对应的航路。

## 实机执行

先确认 `/object_pose` 正在发布、相机和工具 TF 可用，并核实桌面高度与急停状态。
下面的命令使用默认抓取参考偏移 `0 0 -40`，实际闭合位置还受到
`--approach-front-mm 80` 和 `--approach-height-mm 30` 影响；调整时先预览黄色点：

```bash
cd /home/liuqian/copy/test/move_agent/robot_server
source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh
export PYTHONNOUSERSITE=1

/usr/bin/python3 -B -m path_planner.safe_grasp_planner \
  --table-z 0.70 \
  --table-clearance-mm 100 \
  --start-clearance-mm 20 \
  --object-pose-topic /object_pose \
  --control-backend joint \
  --joint-source 2 \
  --settle-time 0.05 \
  --execute
```

`joint` 后端需要 ROS 环境中的 NumPy 1.x 与 Pinocchio。`PYTHONNOUSERSITE=1` 可避免
用户目录中的 NumPy 2.x 覆盖系统包。接管前程序发送当前 22 关节保持目标；执行时固定
躯干、头部和另一侧手臂的目标，只更新选定手臂。到达黄色 `pregrasp` 后闭合夹爪，
再从该点抬升。

## 限制

程序检查桌面平面安全高度、离散路径点、TF 新鲜度和工具位姿误差；尚无完整的自碰撞、
环境网格碰撞、接触力或整机重心稳定性模型。`--max-body-deviation-deg` 只监测躯干
关节角，不能识别底盘或整机侧倾。
