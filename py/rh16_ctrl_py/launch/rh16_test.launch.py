#!/usr/bin/env python3
"""
RH16 Test Launch File
启动RH16测试系统 - 支持双手测试
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """生成测试launch描述"""
    
    # 声明launch参数
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='1',
        description='Test mode: 0=raw cmd, 1=angle cmd, 2=end pos cmd'
    )
    
    hand_side_arg = DeclareLaunchArgument(
        'hand_side',
        default_value='left',
        description='Hand side: left, right, both'
    )
    
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
    
    # 获取launch配置
    test_mode = LaunchConfiguration('test_mode')
    hand_side = LaunchConfiguration('hand_side')
    can_interface_left = LaunchConfiguration('can_interface_left')
    can_interface_right = LaunchConfiguration('can_interface_right')
    
    # 包含主系统launch文件
    main_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rh16_ctrl_py'),
                'launch',
                'rh16_full.launch.py'
            ])
        ]),
        launch_arguments={
            'can_interface_left': can_interface_left,
            'can_interface_right': can_interface_right,
            'left_hand': 'true',
            'right_hand': 'true',  # 支持双手测试
            'publish_rate': '100.0',
        }.items()
    )
    
    # 测试节点
    test_node = Node(
        package='rh16_ctrl_py',
        executable='rh16_test_cpp_aligned',
        name='rh16_test',
        parameters=[{
            'test_mode': test_mode,
            'hand_side': hand_side,
        }],
        output='screen'
    )
    
    # 启动信息
    start_info = LogInfo(
        msg=[
            'Starting RH16 Test System...\n',
            'Test Mode: ', test_mode, ' (0=raw cmd, 1=angle cmd, 2=end pos cmd)\n',
            'Hand Side: ', hand_side, '\n',
            'Left Hand CAN: ', can_interface_left, '\n',
            'Right Hand CAN: ', can_interface_right, '\n',
            '\n',
            'Usage Examples:\n',
            '  Single Hand: ros2 launch rh16_ctrl_py rh16_test.launch.py hand_side:=left\n',
            '  Both Hands:  ros2 launch rh16_ctrl_py rh16_test.launch.py hand_side:=both\n',
            '  Different Mode: ros2 launch rh16_ctrl_py rh16_test.launch.py test_mode:=0\n',
            '\n',
            'Monitor Commands:\n',
            '  ros2 topic echo /left_hand/rh16_status\n',
            '  ros2 topic echo /right_hand/rh16_status\n',
            '  candump can0'
        ]
    )
    
    return LaunchDescription([
        # 参数声明
        test_mode_arg,
        hand_side_arg,
        can_interface_left_arg,
        can_interface_right_arg,
        
        # 启动信息
        start_info,
        
        # 主系统
        main_system_launch,
        
        # 测试节点
        test_node,
    ])
