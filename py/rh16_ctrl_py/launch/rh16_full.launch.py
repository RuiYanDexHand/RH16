#!/usr/bin/env python3
"""
RH16 Full System Launch File
启动完整的RH16手部控制系统 - 支持双手控制
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition


def generate_launch_description():
    """生成完整系统launch描述"""
    
    # 声明launch参数
    can_interface_left_arg = DeclareLaunchArgument(
        'can_interface_left',
        default_value='can0',
        description='CAN interface for left hand'
    )
    
    can_interface_right_arg = DeclareLaunchArgument(
        'can_interface_right',
        default_value='can1',
        description='CAN interface for right hand'
    )
    
    left_hand_arg = DeclareLaunchArgument(
        'left_hand',
        default_value='true',
        description='Enable left hand'
    )
    
    right_hand_arg = DeclareLaunchArgument(
        'right_hand',
        default_value='false',
        description='Enable right hand'
    )
    
    use_pinocchio_arg = DeclareLaunchArgument(
        'use_pinocchio',
        default_value='true',
        description='Use Pinocchio for kinematics'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Publishing rate in Hz'
    )
    
    # 获取launch配置
    can_interface_left = LaunchConfiguration('can_interface_left')
    can_interface_right = LaunchConfiguration('can_interface_right')
    left_hand = LaunchConfiguration('left_hand')
    right_hand = LaunchConfiguration('right_hand')
    use_pinocchio = LaunchConfiguration('use_pinocchio')
    publish_rate = LaunchConfiguration('publish_rate')
    
    # ========== 左手节点组 ==========
    left_hand_group = GroupAction([
        PushRosNamespace('left_hand'),
        
        # 左手CAN通信节点
        Node(
            package='rh16_ctrl_py',
            executable='rh16_can_node',
            name='rh16_can_node',
            parameters=[{
                'can_interface': can_interface_left,
                'publish_rate': publish_rate,
            }],
            output='screen',
            condition=IfCondition(left_hand)
        ),
        
        # 左手运动学节点
        Node(
            package='rh16_ctrl_py',
            executable='rh16_kinematics_node',
            name='rh16_kinematics_node',
            parameters=[{
                'use_pinocchio': use_pinocchio,
            }],
            output='screen',
            condition=IfCondition(left_hand)
        ),
        
        # 左手主控制器节点
        Node(
            package='rh16_ctrl_py',
            executable='rh16_ctrl_node',
            name='rh16_ctrl_node',
            parameters=[{
                'publish_rate': publish_rate,
                'left_hand': True,
                'right_hand': False,
            }],
            output='screen',
            condition=IfCondition(left_hand)
        ),
    ])
    
    # ========== 右手节点组 ==========
    right_hand_group = GroupAction([
        PushRosNamespace('right_hand'),
        
        # 右手CAN通信节点
        Node(
            package='rh16_ctrl_py',
            executable='rh16_can_node',
            name='rh16_can_node',
            parameters=[{
                'can_interface': can_interface_right,
                'publish_rate': publish_rate,
            }],
            output='screen',
            condition=IfCondition(right_hand)
        ),
        
        # 右手运动学节点
        Node(
            package='rh16_ctrl_py',
            executable='rh16_kinematics_node',
            name='rh16_kinematics_node',
            parameters=[{
                'use_pinocchio': use_pinocchio,
            }],
            output='screen',
            condition=IfCondition(right_hand)
        ),
        
        # 右手主控制器节点
        Node(
            package='rh16_ctrl_py',
            executable='rh16_ctrl_node',
            name='rh16_ctrl_node',
            parameters=[{
                'publish_rate': publish_rate,
                'left_hand': False,
                'right_hand': True,
            }],
            output='screen',
            condition=IfCondition(right_hand)
        ),
    ])
    
    # 启动信息
    start_info = LogInfo(
        msg=[
            'Starting RH16 Hand Control System...\n',
            'Left Hand: ', left_hand, ' (CAN: ', can_interface_left, ')\n',
            'Right Hand: ', right_hand, ' (CAN: ', can_interface_right, ')\n',
            'Publish Rate: ', publish_rate, ' Hz\n',
            'Use Pinocchio: ', use_pinocchio, '\n',
            '\n',
            'Topics (when both hands enabled):\n',
            '  Left Hand Command:  /left_hand/rh16_command\n',
            '  Left Hand Status:   /left_hand/rh16_status\n',
            '  Right Hand Command: /right_hand/rh16_command\n',
            '  Right Hand Status:  /right_hand/rh16_status\n',
            '\n',
            'Services (when both hands enabled):\n',
            '  Left Hand FK:  /left_hand/rh16_forward_kinematics\n',
            '  Left Hand IK:  /left_hand/rh16_inverse_kinematics\n',
            '  Right Hand FK: /right_hand/rh16_forward_kinematics\n',
            '  Right Hand IK: /right_hand/rh16_inverse_kinematics'
        ]
    )
    
    return LaunchDescription([
        # 参数声明
        can_interface_left_arg,
        can_interface_right_arg,
        left_hand_arg,
        right_hand_arg,
        use_pinocchio_arg,
        publish_rate_arg,
        
        # 启动信息
        start_info,
        
        # 节点组
        left_hand_group,
        right_hand_group,
    ])
