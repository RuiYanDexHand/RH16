# RH16 Python版本 - 快速开始

## 简单4步运行

```bash
# 1. 进入目录
cd ryhand_sdk/py

# 2. 设置ROS环境
source /opt/ros/humble/setup.bash

# 3. 构建包
colcon build

# 4. 设置工作空间环境
source install/setup.bash
```

## 运行控制

```bash
# 控制手部 (自动配置CAN + 启动控制)
ros2 run rh16_ctrl_py rh_ctrl

# 测试正弦运动 (自动配置CAN + 启动测试)
ros2 run rh16_ctrl_py rh_test

#  通过launch文件同时控制两只手的运动 
ros2 launch rh16_ctrl_py dual_hand_manual_can.launch.py
```

## 就这么简单！

程序会自动：
- ✅ 检测USB转CAN设备 (`/dev/ttyACM*`)
- ✅ 配置SLCAN接口 (`slcand -s8`)
- ✅ 启动CAN通信 (`ifconfig can0 up`)
- ✅ 开始控制16自由度手部

## 监控命令

```bash
# 查看CAN消息
candump can0

# 查看ROS话题
ros2 topic list
ros2 topic echo ryhand_cmd

# 查看节点
ros2 node list
```

## 测试参数

```bash
# 左手测试 (默认)
ros2 run rh16_ctrl_py rh_test

# 右手测试
ros2 run rh16_ctrl_py rh_test --ros-args -p hand_side:=right

# 不同模式
ros2 run rh16_ctrl_py rh_test --ros-args -p test_mode:=0  # 原始电机
ros2 run rh16_ctrl_py rh_test --ros-args -p test_mode:=1  # 关节角度 (默认)
```

## 故障排除

### USB设备未找到
```bash
# 检查设备
ls /dev/ttyACM*

# WSL用户需要在Windows中共享USB
# Windows PowerShell (管理员):
# usbipd list
# usbipd bind -b 4-3  
# usbipd attach -b 4-3 -w
```

### 权限问题
```bash
sudo usermod -a -G dialout $USER
# 重新登录
```

### 禁用自动CAN配置
```bash
ros2 run rh16_ctrl_py rh_ctrl --ros-args -p auto_can_setup:=false
```

## 成功标志

看到这些输出说明运行成功：
```
[INFO] [rh16_ctrl_node]: RH16主控制器启动
[INFO] [rh16_ctrl_node]: 开始自动配置CAN接口...
找到TTY设备: /dev/ttyACM0
CAN接口配置成功!
[INFO] [rh16_ctrl_node]: CAN通信初始化成功
[INFO] [rh16_ctrl_node]: RH16主控制器初始化完成
```

现在您可以用最简单的命令控制16自由度手部了！
