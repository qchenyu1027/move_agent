# robot_server

机器人端服务。`agent_tools/` 提供 FastAPI（`/health`、`/move`、`/grasp`、`/tasks`、`/cancel`）与任务调度，
`tools/` 提供 move / grasp 工具封装和控制脚本（`tools/scripts/`）。上层 move_agent 通过 HTTP 调用它，
详见 [`agent_tools/README.md`](agent_tools/README.md) 和 [`agent_tools/README_LLM_API.md`](agent_tools/README_LLM_API.md)。

**所有命令都必须在本目录（`robot_server/`）下执行。** 这里的 `agent_tools` 和 `tools` 是按这两个名字相互导入的包；
仓库根目录另有一个同名的 `tools/`（move_agent 提供给 LLM 的工具），不在本目录运行会找不到包，或导入到错误的 `tools`。

```bash
# 在机器人电脑上启动服务（需要 ROS 环境与 ROBOT_API_KEY，见 agent_tools/README.md）
cd robot_server && agent_tools/start_api.sh

# 离线测试（不连接机器人）
cd robot_server && python3 -B -m unittest -v agent_tools.test_agent_tools agent_tools.test_agent_tools_api agent_tools.test_base_move_world
```
