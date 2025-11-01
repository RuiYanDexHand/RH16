#!/bin/bash

# 快速重新构建脚本
echo "重新构建RH16 Python包..."

# 设置ROS2环境
source /opt/ros/humble/setup.bash

# 清理旧的构建
rm -rf build install log

# 重新构建
colcon build --packages-select rh16_msg_py rh16_cmd_py rh16_ctrl_py

# 设置环境
source install/setup.bash

echo "构建完成！现在可以运行:"
echo "ros2 run rh16_ctrl_py rh_ctrl"
echo "ros2 run rh16_ctrl_py rh_test"
