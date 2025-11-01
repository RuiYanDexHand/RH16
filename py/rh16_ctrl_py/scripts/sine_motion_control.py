#!/usr/bin/env python3
"""
RH16 Sine Motion Control Script
RH16正弦运动控制脚本 - 连接真实硬件进行正弦运动
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import time
import numpy as np

from rh16_cmd_py.msg import Rh16Cmd
from rh16_msg_py.msg import Rh16Msg


class SineMotionController(Node):
    """正弦运动控制器"""
    
    def __init__(self):
        super().__init__('sine_motion_controller')
        
        # 声明参数
        self.declare_parameter('hand_side', 'left')  # left, right, both
        self.declare_parameter('frequency', 0.5)     # 频率 (Hz)
        self.declare_parameter('amplitude', 45.0)    # 幅度 (度)
        self.declare_parameter('fingers', [0, 1, 2, 3, 4])  # 控制的手指 (0-4)
        self.declare_parameter('joints', [1, 2])     # 控制的关节 (1-3)
        self.declare_parameter('phase_offset', 0.0)  # 相位偏移 (弧度)
        self.declare_parameter('control_rate', 50.0) # 控制频率 (Hz)
        
        # 获取参数
        self.hand_side = self.get_parameter('hand_side').get_parameter_value().string_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        self.fingers = self.get_parameter('fingers').get_parameter_value().integer_array_value
        self.joints = self.get_parameter('joints').get_parameter_value().integer_array_value
        self.phase_offset = self.get_parameter('phase_offset').get_parameter_value().double_value
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        
        self.get_logger().info(f'正弦运动控制器启动:')
        self.get_logger().info(f'  手部: {self.hand_side}')
        self.get_logger().info(f'  频率: {self.frequency} Hz')
        self.get_logger().info(f'  幅度: {self.amplitude}°')
        self.get_logger().info(f'  控制手指: {self.fingers}')
        self.get_logger().info(f'  控制关节: {self.joints}')
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 发布器
        if self.hand_side == 'both':
            self.cmd_pub_left = self.create_publisher(
                Rh16Cmd, '/left_hand/rh16_command', qos_profile)
            self.cmd_pub_right = self.create_publisher(
                Rh16Cmd, '/right_hand/rh16_command', qos_profile)
        elif self.hand_side == 'left':
            self.cmd_pub_left = self.create_publisher(
                Rh16Cmd, '/left_hand/rh16_command', qos_profile)
        else:  # right
            self.cmd_pub_right = self.create_publisher(
                Rh16Cmd, '/right_hand/rh16_command', qos_profile)
        
        # 订阅器 - 监控状态
        if self.hand_side in ['left', 'both']:
            self.status_sub_left = self.create_subscription(
                Rh16Msg, '/left_hand/rh16_status',
                lambda msg: self.status_callback(msg, 'left'), qos_profile)
        
        if self.hand_side in ['right', 'both']:
            self.status_sub_right = self.create_subscription(
                Rh16Msg, '/right_hand/rh16_status',
                lambda msg: self.status_callback(msg, 'right'), qos_profile)
        
        # 状态变量
        self.start_time = time.time()
        self.current_status_left = None
        self.current_status_right = None
        
        # 控制定时器
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self.control_callback)
        
        # 状态打印定时器
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('正弦运动控制开始...')
    
    def deg_to_rad(self, deg):
        """角度转弧度"""
        return deg * math.pi / 180.0
    
    def rad_to_deg(self, rad):
        """弧度转角度"""
        return rad * 180.0 / math.pi
    
    def control_callback(self):
        """控制回调 - 生成正弦运动"""
        current_time = time.time() - self.start_time
        
        # 计算正弦值
        sine_value = math.sin(2 * math.pi * self.frequency * current_time + self.phase_offset)
        
        # 计算目标角度 (弧度)
        target_angle_rad = self.deg_to_rad(self.amplitude * sine_value)
        
        # 确保角度为正值 (手指只能向内弯曲)
        target_angle_rad = max(0.0, target_angle_rad)
        
        # 发布命令
        if self.hand_side in ['left', 'both']:
            self.publish_command('left', target_angle_rad)
        
        if self.hand_side in ['right', 'both']:
            self.publish_command('right', target_angle_rad)
    
    def publish_command(self, hand, target_angle):
        """发布控制命令"""
        cmd = Rh16Cmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.lr = (hand == 'right')
        cmd.mode = 1  # 关节角度控制模式
        
        # 初始化关节角度
        cmd.j_ang = [0.0] * 20
        
        # 设置目标手指和关节的角度
        for finger_idx in self.fingers:
            if 0 <= finger_idx <= 4:
                for joint_idx in self.joints:
                    if 1 <= joint_idx <= 3:  # 关节1,2,3
                        array_idx = finger_idx * 4 + joint_idx
                        if array_idx < 20:
                            # 根据关节类型调整幅度
                            if joint_idx == 1:
                                cmd.j_ang[array_idx] = target_angle
                            elif joint_idx == 2:
                                cmd.j_ang[array_idx] = target_angle * 0.8  # 第三关节幅度稍小
                            else:  # joint_idx == 3
                                cmd.j_ang[array_idx] = target_angle * 0.6  # 第四关节幅度更小
        
        # 设置电机参数
        cmd.m_spd = [1000] * 16      # 中等速度
        cmd.m_curlimit = [80] * 16   # 电流限制
        
        # 发布命令
        if hand == 'left' and hasattr(self, 'cmd_pub_left'):
            self.cmd_pub_left.publish(cmd)
        elif hand == 'right' and hasattr(self, 'cmd_pub_right'):
            self.cmd_pub_right.publish(cmd)
    
    def status_callback(self, msg, hand):
        """状态回调"""
        if hand == 'left':
            self.current_status_left = msg
        else:
            self.current_status_right = msg
    
    def print_status(self):
        """打印状态信息"""
        current_time = time.time() - self.start_time
        sine_value = math.sin(2 * math.pi * self.frequency * current_time + self.phase_offset)
        target_angle_deg = self.amplitude * sine_value
        
        self.get_logger().info(f'时间: {current_time:.1f}s, 目标角度: {target_angle_deg:.1f}°')
        
        # 打印实际关节角度
        if self.current_status_left:
            angles = [self.rad_to_deg(a) for a in self.current_status_left.j_ang[:8]]
            self.get_logger().info(f'左手关节角度: {[f"{a:.1f}°" for a in angles]}')
        
        if self.current_status_right:
            angles = [self.rad_to_deg(a) for a in self.current_status_right.j_ang[:8]]
            self.get_logger().info(f'右手关节角度: {[f"{a:.1f}°" for a in angles]}')


def main(args=None):
    rclpy.init(args=args)
    
    controller = SineMotionController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('正弦运动控制停止')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
