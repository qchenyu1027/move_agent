#!/usr/bin/env python3
# 用途：通过 ROS select 服务依次选择关节命令源和 IK 请求源；运行：source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh && /usr/bin/python3 tools/scripts/select_control_sources.py --ik-source 2 --joint-source 0。
"""切换控制 mux，检查服务返回的实际源编号。"""

import argparse
import math
import os
import sys

try:  # --help 和离线预览不要求已加载 ROS 环境。
    import rclpy
    from marvin_msgs.srv import Int
except ImportError as error:  # pragma: no cover - 取决于运行环境是否加载 ROS
    rclpy = None
    Int = None
    ROS_IMPORT_ERROR = error
else:
    ROS_IMPORT_ERROR = None


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ik-source', type=int, required=True, help='目标 IK 请求源编号')
    parser.add_argument('--joint-source', type=int, required=True, help='目标关节命令源编号')
    parser.add_argument('--namespace', default=os.environ.get('APEX_ROS_NAMESPACE', 'tj'),
                        help='ROS 命名空间，默认 APEX_ROS_NAMESPACE 或 tj')
    parser.add_argument('--wait-timeout-s', type=float, default=10.0,
                        help='等待每个服务及其响应的超时时间（秒），默认 10')
    return parser.parse_args(argv)


def select_source(node, service, source, timeout):
    """服务确认的活动源必须等于请求值，否则不执行后续步骤。"""
    client = node.create_client(Int, service)
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f'等待 ROS 服务超时：{service}')
    request = Int.Request()
    request.data = source
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError(f'ROS 服务响应超时：{service}')
    response = future.result()
    if response is None:
        raise RuntimeError(f'ROS 服务没有返回响应：{service}')
    if not response.success or response.error_code != source:
        raise RuntimeError(f'{service} 切换失败：{response.message}；实际源={response.error_code}')
    node.get_logger().info(f'{service} 已选择源 {source}：{response.message}')


def main(argv=None):
    args = parse_args(argv)
    if args.ik_source < 0 or args.joint_source < 0:
        print('--ik-source 和 --joint-source 必须是非负整数', file=sys.stderr)
        return 2
    if not math.isfinite(args.wait_timeout_s) or args.wait_timeout_s <= 0:
        print('--wait-timeout-s 必须是有限正数', file=sys.stderr)
        return 2
    if rclpy is None:
        print(f'无法导入 ROS rclpy/marvin_msgs，请先加载 ROS 环境：{ROS_IMPORT_ERROR}',
              file=sys.stderr)
        return 2

    namespace = args.namespace.strip('/')
    prefix = f'/{namespace}' if namespace else ''
    node = None
    initialized = False
    try:
        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node('select_control_sources_tool')
        select_source(node, f'{prefix}/control/joint_cmd_mux/select',
                      args.joint_source, args.wait_timeout_s)
        select_source(node, f'{prefix}/control/ik_request_mux/select',
                      args.ik_source, args.wait_timeout_s)
        return 0
    except (RuntimeError, ValueError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(str(error), file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
