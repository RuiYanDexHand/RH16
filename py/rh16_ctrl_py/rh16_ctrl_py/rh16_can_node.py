#!/usr/bin/env python3
"""
RH16 CAN Communication Node
RH16 CAN通信节点 - 负责与硬件的CAN通信
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import socket
import struct
import threading
import time
from typing import List, Tuple, Optional

from std_msgs.msg import Int32MultiArray, Float64MultiArray
from rh16_cmd_py.msg import Rh16Cmd


class CANSocket:
    """CAN套接字通信类"""
    
    def __init__(self, interface="can0"):
        self.interface = interface
        self.socket = None
        self.is_connected = False
        
    def open(self):
        """打开CAN套接字"""
        try:
            print(f"创建CAN套接字，接口: {self.interface}")

            # 创建CAN套接字
            self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)

            # 绑定到CAN接口
            self.socket.bind((self.interface,))

            # 设置发送为阻塞模式，接收为非阻塞模式
            # 发送时不设置超时，避免timed out错误
            # 只在接收时设置超时

            # 测试套接字是否真正工作
            print(f"CAN套接字绑定成功: {self.interface}")
            print(f"套接字文件描述符: {self.socket.fileno()}")

            self.is_connected = True
            return True

        except Exception as e:
            print(f"CAN接口打开失败: {e}")
            self.is_connected = False
            return False
    
    def close(self):
        """关闭CAN套接字"""
        if self.socket:
            self.socket.close()
            self.socket = None
        self.is_connected = False
    
    def send_message(self, can_id: int, data: bytes) -> bool:
        """发送CAN消息"""
        if not self.is_connected or not self.socket:
            print(f"CAN未连接，无法发送消息: ID=0x{can_id:02X}")
            return False

        try:
            # 确保发送时没有超时限制
            self.socket.settimeout(None)  # 阻塞模式，不超时

            # 使用与C++完全相同的CAN帧格式
            # struct can_frame { canid_t can_id; __u8 can_dlc; __u8 __pad; __u8 __res0; __u8 __res1; __u8 data[8]; };
            can_frame = struct.pack("=IBBBB8s",
                                   can_id,           # can_id (4字节)
                                   len(data),        # can_dlc (1字节)
                                   0,                # __pad (1字节)
                                   0,                # __res0 (1字节)
                                   0,                # __res1 (1字节)
                                   data.ljust(8, b'\x00'))  # data[8] (8字节)

            # 发送到CAN总线
            bytes_sent = self.socket.send(can_frame)
            print(f"发送CAN消息: ID=0x{can_id:02X}, 数据={data.hex()}, DLC={len(data)}, 字节数={bytes_sent}")

            return True

        except Exception as e:
            print(f"CAN发送失败: ID=0x{can_id:02X}, 错误={e}")
            return False
    
    def receive_message(self) -> Tuple[bool, int, bytes]:
        """接收CAN消息"""
        if not self.is_connected or not self.socket:
            return False, 0, b''

        try:
            # 临时设置接收超时
            self.socket.settimeout(0.001)  # 1ms超时，只用于接收

            # 接收CAN帧
            can_frame = self.socket.recv(16)
            can_id, length = struct.unpack("=IB3x", can_frame[:8])
            data = can_frame[8:8+length]
            return True, can_id, data

        except socket.timeout:
            return False, 0, b''
        except Exception as e:
            return False, 0, b''


class ServoData:
    """电机数据结构"""
    def __init__(self):
        self.pos = 0      # 位置
        self.spd = 0      # 速度
        self.cur = 0      # 电流
        self.force = 0    # 力
        self.status = 0   # 状态


class RH16CANNode(Node):
    """RH16 CAN通信节点"""
    
    def __init__(self):
        super().__init__('rh16_can_node')
        
        # 声明参数
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('publish_rate', 100.0)  # 100Hz
        
        # 获取参数
        self.can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        self.get_logger().info(f'RH16 CAN节点启动，接口: {self.can_interface}')
        
        # 初始化CAN通信
        self.can_socket = CANSocket(self.can_interface)
        
        # 电机数据
        self.servo_data_w = [ServoData() for _ in range(16)]  # 写入数据
        self.servo_data_r = [ServoData() for _ in range(16)]  # 读取数据
        
        # 初始化电机数据
        for i in range(16):
            self.servo_data_w[i].pos = 2048
            self.servo_data_w[i].spd = 1000
            self.servo_data_w[i].cur = 80
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 发布器
        self.motor_status_pub = self.create_publisher(
            Int32MultiArray, 
            'motor_status', 
            qos_profile
        )
        
        self.motor_positions_pub = self.create_publisher(
            Int32MultiArray, 
            'motor_positions', 
            qos_profile
        )
        
        self.motor_currents_pub = self.create_publisher(
            Int32MultiArray, 
            'motor_currents', 
            qos_profile
        )
        
        # 订阅器
        self.motor_cmd_sub = self.create_subscription(
            Int32MultiArray,
            'motor_commands',
            self.motor_cmd_callback,
            qos_profile
        )
        
        # 定时器
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, 
            self.publish_status
        )
        
        # CAN接收线程
        self.thread_go = True
        self.can_thread = None
        
        # 启动CAN通信
        self.start_can()
    
    def start_can(self):
        """启动CAN通信"""
        if not self.can_socket.open():
            self.get_logger().error("CAN接口打开失败")
            return False
        
        # 清除电机故障
        for i in range(1, 16):
            self.clear_fault(i)
            time.sleep(0.001)
        
        # 启动CAN接收线程
        self.thread_go = True
        self.can_thread = threading.Thread(target=self._can_rx_thread, daemon=True)
        self.can_thread.start()
        
        self.get_logger().info("CAN通信启动成功")
        return True
    
    def stop_can(self):
        """停止CAN通信"""
        self.thread_go = False
        if self.can_thread:
            self.can_thread.join(timeout=1.0)
        self.can_socket.close()
        self.get_logger().info("CAN通信已停止")
    
    def _can_rx_thread(self):
        """CAN接收线程"""
        self.get_logger().info("CAN接收线程启动")
        
        while self.thread_go:
            # 接收CAN消息
            success, can_id, data = self.can_socket.receive_message()
            if success:
                self.process_received_message(can_id, data)
            
            # 短暂休眠
            time.sleep(0.0001)  # 100微秒
        
        self.get_logger().info("CAN接收线程结束")
    
    def servo_move_mix(self, motor_id: int, pos: int, spd: int, cur: int) -> bool:
        """电机混合运动控制 - 力位混合控制 (对应C++的RyMotion_ServoMove_Mix)"""
        if not self.can_socket.is_connected:
            return False

        # 构造CAN消息 - 根据C++库的协议格式
        # 参数: ucId, sTargetAngle(0-4095), usRunSpeed(0-65535), sMaxCurrent(-1000到1000)
        can_id = motor_id  # 电机ID (1-16)

        # 确保参数范围正确
        pos = max(0, min(4095, pos))      # 位置范围 0-4095
        spd = max(0, min(65535, spd))     # 速度范围 0-65535
        cur = max(-1000, min(1000, cur))  # 电流范围 -1000到1000

        # CAN数据格式: 命令码(1字节) + 位置(2字节) + 速度(2字节) + 电流(2字节)
        data = struct.pack('<BHHH', 0xAA, pos, spd, cur & 0xFFFF)  # 处理负数电流

        print(f"电机{motor_id}混合控制: 位置={pos}, 速度={spd}, 电流={cur}")
        return self.can_socket.send_message(can_id, data)
    
    def clear_fault(self, motor_id: int) -> bool:
        """清除电机故障"""
        if not self.can_socket.is_connected:
            return False
        
        can_id = motor_id
        data = struct.pack('<B', 0xA1)  # 清除故障命令
        
        return self.can_socket.send_message(can_id, data)
    
    def process_received_message(self, can_id: int, data: bytes):
        """处理接收到的CAN消息"""
        if len(data) < 1:
            return
        
        cmd = data[0]
        motor_id = can_id - 1  # 假设CAN ID从1开始
        
        if 0 <= motor_id < 16:
            if cmd in [0xA0, 0xA1, 0xA6, 0xA9, 0xAA]:
                # 解析电机状态数据
                if len(data) >= 8:
                    self.servo_data_r[motor_id].status = data[1]
                    self.servo_data_r[motor_id].pos = struct.unpack('<H', data[2:4])[0]
                    self.servo_data_r[motor_id].spd = struct.unpack('<h', data[4:6])[0]
                    self.servo_data_r[motor_id].cur = struct.unpack('<h', data[6:8])[0]
                    
                    # 处理负数
                    if self.servo_data_r[motor_id].spd > 2047:
                        self.servo_data_r[motor_id].spd -= 4096
                    if self.servo_data_r[motor_id].cur > 2047:
                        self.servo_data_r[motor_id].cur -= 4096
    
    def motor_cmd_callback(self, msg):
        """电机命令回调"""
        # 期望格式: [motor_id, pos, spd, cur, motor_id, pos, spd, cur, ...]
        data = msg.data
        
        for i in range(0, len(data), 4):
            if i + 3 < len(data):
                motor_id = data[i]
                pos = data[i + 1]
                spd = data[i + 2]
                cur = data[i + 3]
                
                if 1 <= motor_id <= 16:
                    self.servo_move_mix(motor_id, pos, spd, cur)
                    
                    # 更新写入数据
                    idx = motor_id - 1
                    self.servo_data_w[idx].pos = pos
                    self.servo_data_w[idx].spd = spd
                    self.servo_data_w[idx].cur = cur
    
    def publish_status(self):
        """发布电机状态"""
        # 发布电机状态
        status_msg = Int32MultiArray()
        status_msg.data = [servo.status for servo in self.servo_data_r]
        self.motor_status_pub.publish(status_msg)
        
        # 发布电机位置
        pos_msg = Int32MultiArray()
        pos_msg.data = [servo.pos for servo in self.servo_data_r]
        self.motor_positions_pub.publish(pos_msg)
        
        # 发布电机电流
        cur_msg = Int32MultiArray()
        cur_msg.data = [servo.cur for servo in self.servo_data_r]
        self.motor_currents_pub.publish(cur_msg)
    
    def destroy_node(self):
        """销毁节点"""
        self.stop_can()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = RH16CANNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
