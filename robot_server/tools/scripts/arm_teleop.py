#!/usr/bin/env python3
# 用途：从 tools/scripts 启动左右臂 ROS 笛卡尔遥操或一次性位移；运行：source /home/liuqian/WBC/Apex_Deploy/apex_ros_env.sh && PYTHONNOUSERSITE=1 /usr/bin/python3 tools/scripts/arm_teleop.py --side left --execute --move-z-mm 50。
"""左右臂 ROS 笛卡尔遥操的 tools/scripts 启动入口。"""

from pathlib import Path
import runpy


ROOT_SCRIPT = Path(__file__).resolve().with_name('right_arm_teleop.py')


def main():
    if not ROOT_SCRIPT.is_file():
        raise FileNotFoundError(f'找不到遥操实现：{ROOT_SCRIPT}')
    runpy.run_path(str(ROOT_SCRIPT), run_name='__main__')


if __name__ == '__main__':
    main()
