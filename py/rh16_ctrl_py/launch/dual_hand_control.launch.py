#!/usr/bin/env python3
"""
Launch a dual-hand control system for the RH16 robot hand.

This launch file starts two sets of nodes, one for the left hand and one for the right hand.
Each set consists of a controller node and a test node.

- Left Hand:  Uses can0, controlled via left_hand/... topics.
- Right Hand: Uses can1, controlled via right_hand/... topics.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for dual-hand control.
    
    Returns:
        LaunchDescription: The launch description object.
    """
    
    return LaunchDescription([
        
        # === Left Hand Nodes ===
        
        # 1. Left Hand Controller Node
        Node(
            package='rh16_ctrl_py',
            executable='rh_ctrl',
            name='left_hand_controller',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'can_interface': 'can0'},
                {'hand_type': 'left'},
                {'cmd_topic': 'left_hand/ryhand_cmd'},
                {'status_topic': 'left_hand/ryhand_status'}
            ]
        ),
        
        # 2. Left Hand Test Node
        Node(
            package='rh16_ctrl_py',
            executable='rh_test',
            name='left_hand_test',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'hand_type': 'left'},
                {'cmd_topic': 'left_hand/ryhand_cmd'}
            ]
        ),
        
        # === Right Hand Nodes ===
        
        # 3. Right Hand Controller Node
        Node(
            package='rh16_ctrl_py',
            executable='rh_ctrl',
            name='right_hand_controller',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'can_interface': 'can1'},
                {'hand_type': 'right'},
                {'cmd_topic': 'right_hand/ryhand_cmd'},
                {'status_topic': 'right_hand/ryhand_status'}
            ]
        ),
        
        # 4. Right Hand Test Node
        Node(
            package='rh16_ctrl_py',
            executable='rh_test',
            name='right_hand_test',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'hand_type': 'right'},
                {'cmd_topic': 'right_hand/ryhand_cmd'}
            ]
        ),
    ])

