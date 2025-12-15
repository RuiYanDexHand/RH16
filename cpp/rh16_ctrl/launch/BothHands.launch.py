# 左右手双节点启动文件（ROS 2）
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rh16_ctrl',
            executable='rh_ctrl',
            namespace='rh16_left',
            name='left',
            output='screen',
            arguments=['can0']   # argv[1] = 'can0'
        ),
        Node(
            package='rh16_ctrl',
            executable='rh_ctrl',
            namespace='rh16_right',
            name='right',
            output='screen',
            arguments=['can1']   # argv[1] = 'can0'
        ),
        # ros2 run rh16_ctrl rh_ctrl can0
    ])


# ros2 launch rh16_ctrl BothHands.launch.py

# 运行该脚本应该能看到如下输出：

# $ ros2 topic list
# *
# /rh16_left/ryhand_cmd
# /rh16_left/ryhand_status
# /rh16_right/ryhand_cmd
# /rh16_right/ryhand_status
# *

# $ ros2 node list
# /rh16_left/left
# /rh16_right/right