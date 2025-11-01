#!/bin/bash
# RH16 Python ROS Packages Build Script
# RH16 Python ROS包构建脚本

set -e

echo "=========================================="
echo "Building RH16 Python ROS Packages"
echo "构建RH16 Python ROS包"
echo "=========================================="

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS2环境未设置"
    echo "请先source ROS2环境: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "ROS2发行版: $ROS_DISTRO"

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

echo "工作空间: $WORKSPACE_DIR"

# 检查依赖
echo "检查Python依赖..."
python3 -c "import numpy" || {
    echo "错误: numpy未安装"
    echo "请安装: pip3 install numpy"
    exit 1
}

echo "检查colcon..."
which colcon > /dev/null || {
    echo "错误: colcon未安装"
    echo "请安装: sudo apt install python3-colcon-common-extensions"
    exit 1
}

# 进入工作空间
cd "$WORKSPACE_DIR"

# 清理之前的构建
echo "清理之前的构建..."
rm -rf build install log

# 构建包
echo "开始构建..."
colcon build \
    --packages-select rh16_msg_py rh16_cmd_py rh16_ctrl_py \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers $(nproc)

# 检查构建结果
if [ $? -eq 0 ]; then
    echo "=========================================="
    echo "构建成功!"
    echo "=========================================="
    
    echo "设置环境:"
    echo "source $WORKSPACE_DIR/install/setup.bash"
    echo ""
    
    echo "运行测试:"
    echo "ros2 launch rh16_ctrl_py rh16_test.launch.py"
    echo ""
    
    echo "运行完整系统:"
    echo "ros2 launch rh16_ctrl_py rh16_full.launch.py"
    echo ""
    
    echo "查看话题:"
    echo "ros2 topic list"
    echo "ros2 topic echo /rh16_status"
    echo ""
    
    echo "调用服务:"
    echo "ros2 service list"
    echo "ros2 service call /rh16_forward_kinematics rh16_msg_py/srv/Rh16Fk"
    
else
    echo "=========================================="
    echo "构建失败!"
    echo "=========================================="
    exit 1
fi
