#!/bin/bash
set -e

# 激活虚拟环境
source /opt/venv/bin/activate

# 设置 ROS 2 环境变量
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

# 将虚拟环境的 Python 库添加到 PYTHONPATH
export PYTHONPATH="/opt/venv/lib/python3.10/site-packages:$PYTHONPATH"

# 自動啟動
# ros2 launch robot_dog_launch robot_dog_launch.py

# 执行传递的命令
exec "$@"
