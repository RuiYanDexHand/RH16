#!/usr/bin/env python3
"""
Launch a dual-hand control system with manual CAN setup.

This launch file is a simplified version that disables the automatic CAN setup feature.
It assumes that the user has already configured the can0 and can1 interfaces manually.

- Left Hand:  Uses can0, controlled via left_hand/... topics.
- Right Hand: Uses can1, controlled via right_hand/... topics.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for dual-hand control with manual CAN setup.
    """
    
    return LaunchDescription([
        
        # === Left Hand Nodes ===
        
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
                {'status_topic': 'left_hand/ryhand_status'},
                {'auto_can_setup': False}  # <-- Disable automatic setup
            ]
        ),
        
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
                {'status_topic': 'right_hand/ryhand_status'},
                {'auto_can_setup': False}  # <-- Disable automatic setup
            ]
        ),
        
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

