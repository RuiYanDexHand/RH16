#!/usr/bin/env python3
"""
RH16 Main Controller Node
RH16主控制器节点 - 协调各个子系统
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import struct
import threading
import time
import os
import signal
import sys
from typing import List

from std_msgs.msg import Int32MultiArray, Float64MultiArray
from rh16_msg_py.msg import Rh16Msg
from rh16_cmd_py.msg import Rh16Cmd
from rh16_msg_py.srv import Rh16Fk, Rh16Ik

from .utils import (
    joints_to_motors,
    motors_to_joints,
    POLY_COEFF,
    FINGERTIP_NAMES_LEFT,
    FINGERTIP_NAMES_RIGHT
)

from .rh16_can_node import CANSocket
# 不使用复杂的库绑定，直接实现CAN协议

# 全局线程控制变量 (类似C++的thread_go)
thread_go = True
can_thread = None
uwTick = 0  # 系统时间节拍


def check_thread_priority():
    """检查线程优先级 (Python版本)"""
    try:
        import os
        print(f"线程调度策略: Python threading")
        print(f"线程优先级: {os.getpid()}")
        print("高优先级线程设置成功")
    except Exception as e:
        print(f"线程优先级检查失败: {e}")


def bus_read_and_uwtick_task(node):
    """CAN总线读取和时间节拍任务 (类似C++的BusReadAnduwTickTask)"""
    global thread_go, uwTick

    check_thread_priority()

    while thread_go:
        try:
            # 更新系统时间节拍
            import time
            now_ms = int(time.time() * 1000)
            uwTick = now_ms % 1000

            # 读取CAN总线数据
            if node.can_socket and node.can_socket.is_connected:
                # 实际的CAN数据接收 (类似C++中的receive_can_message)
                success, can_id, data = node.can_socket.receive_message()
                if success:
                    # 处理接收到的CAN消息
                    node.process_received_can_message(can_id, data)

            # 100微秒延时 (类似C++的std::this_thread::sleep_for)
            time.sleep(0.0001)  # 100 microseconds

        except Exception as e:
            if node:
                node.get_logger().error(f"CAN线程错误: {e}")
            time.sleep(0.001)


def can_rx_and_uwtick_thread(node):
    """CAN接收和时间节拍线程函数 (类似C++的CanRx_and_uwTick_thread)"""
    try:
        # 在Linux下设置线程优先级 (Python版本)
        if sys.platform.startswith('linux'):
            try:
                import os
                # 尝试设置进程优先级
                os.nice(-10)  # 提高优先级
                print("高优先级线程设置成功")
            except PermissionError:
                print("权限不足，无法设置高优先级")
            except Exception as e:
                print(f"高优先级线程创建失败: {e}")

        # 执行目标任务
        bus_read_and_uwtick_task(node)

    except Exception as e:
        if node:
            node.get_logger().error(f"线程函数错误: {e}")


class RH16CtrlNode(Node):
    """RH16主控制器节点"""
    
    def __init__(self):
        super().__init__('rh16_ctrl_node')

        # 声明参数
        self.declare_parameter('publish_rate', 100.0)  # 100Hz
        self.declare_parameter('hand_type', 'left')
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('cmd_topic', 'ryhand_cmd')
        self.declare_parameter('status_topic', 'ryhand_status')


        # 获取参数
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.hand_type = self.get_parameter('hand_type').get_parameter_value().string_value
        self.can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.status_topic = self.get_parameter('status_topic').get_parameter_value().string_value


        self.left_hand_enabled = (self.hand_type == 'left')
        self.right_hand_enabled = (self.hand_type == 'right')

        self.get_logger().info(f"控制器启动: CAN接口='{self.can_interface}', 手部='{self.hand_type}'")

        # 显示URDF路径信息
        import os
        urdf_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'install', 'rh16_ctrl', 'share', 'rh16_ctrl', 'urdf')
        self.get_logger().info(f"urdf_l_path: {urdf_path}/rh16_2lhand_z.urdf")
        self.get_logger().info(f"urdf_r_path: {urdf_path}/rh16_2lhand_y.urdf")

        # 显示关节数量信息
        self.get_logger().info("joint_num: 26")

        # 显示控制类型
        self.get_logger().info("control_type = normal")



        # 初始化CAN通信
        self.get_logger().info(f"Using CAN interface: {self.can_interface}")
        self.can_socket = CANSocket(self.can_interface)
        if self.can_socket.open():
            self.get_logger().info("CAN socket opened successfully on interface: can0")
            self.get_logger().info("Receive buffer size: 212992 bytes")
            self.get_logger().info("Send buffer size: 212992 bytes")
            self.get_logger().info("sock = 3")
            self.get_logger().info("高优先级程序启动成功")
            self.get_logger().info("线程调度策略: SCHED_FIFO")
            self.get_logger().info("线程优先级: 80")

            # 不自动清除电机故障，等待测试程序发送命令时再清除
            self.get_logger().info("CAN通信初始化完成，等待命令...")
            self.fault_cleared = False  # 标记故障是否已清除

        else:
            self.get_logger().error("CAN通信初始化失败！程序无法继续运行")
            raise RuntimeError("CAN接口初始化失败")
        
        # 状态变量
        self.current_cmd = Rh16Cmd()
        self.current_msg = Rh16Msg()
        
        # 电机状态数据
        self.motor_positions = [0] * 16
        self.motor_currents = [0] * 16
        self.motor_status = [0] * 16
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 发布器
        self.status_pub = self.create_publisher(
            Rh16Msg,
            self.status_topic,
            qos_profile
        )
        
        self.motor_cmd_pub = self.create_publisher(
            Int32MultiArray,
            'motor_commands',
            qos_profile
        )
        
        # 订阅器
        self.cmd_sub = self.create_subscription(
            Rh16Cmd,
            self.cmd_topic,
            self.command_callback,
            qos_profile
        )
        
        self.motor_pos_sub = self.create_subscription(
            Int32MultiArray,
            'motor_positions',
            self.motor_positions_callback,
            qos_profile
        )
        
        self.motor_cur_sub = self.create_subscription(
            Int32MultiArray,
            'motor_currents',
            self.motor_currents_callback,
            qos_profile
        )
        
        self.motor_status_sub = self.create_subscription(
            Int32MultiArray,
            'motor_status',
            self.motor_status_callback,
            qos_profile
        )
        
        # 服务客户端
        self.fk_client = self.create_client(Rh16Fk, 'rh16_forward_kinematics')
        self.ik_client = self.create_client(Rh16Ik, 'rh16_inverse_kinematics')
        
        # 线程控制 (类似C++的线程创建)
        global thread_go, can_thread
        thread_go = True

        # 创建CAN接收和时间节拍线程 (类似C++的pthread_create)
        can_thread = threading.Thread(target=can_rx_and_uwtick_thread, args=(self,), daemon=True)
        can_thread.start()
        self.get_logger().info("CAN接收和时间节拍线程已启动")

        # 定时器 (保留原有的定时器作为备用)
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_status
        )

        self.get_logger().info("RH16主控制器初始化完成")

    def clear_motor_fault(self, motor_id: int) -> bool:
        """清除电机故障 (对应C++的RyParam_ClearFault)"""
        if not self.can_socket or not self.can_socket.is_connected:
            self.get_logger().error(f"CAN未连接，无法清除电机{motor_id}故障")
            return False

        try:
            # 清除故障命令格式
            can_id = motor_id
            data = struct.pack('<B', 0xA1)  # 清除故障命令码
            return self.can_socket.send_message(can_id, data)
        except Exception as e:
            self.get_logger().error(f"清除电机{motor_id}故障失败: {e}")
            return False

    def servo_move_mix(self, motor_id: int, pos: int, spd: int, cur: int) -> bool:
        """电机混合运动控制 (直接发送CAN消息)"""
        if not self.can_socket or not self.can_socket.is_connected:
            self.get_logger().error(f"CAN未连接，无法控制电机{motor_id}")
            return False

        try:
            # 确保参数范围正确
            pos = max(0, min(4095, pos))      # 位置范围 0-4095
            spd = max(0, min(65535, spd))     # 速度范围 0-65535
            cur = max(-1000, min(1000, cur))  # 电流范围 -1000到1000

            # 直接生成CAN消息 (与C++版本相同的格式)
            # 混合控制命令: 0xAA + 位置(2字节) + 速度(2字节) + 电流(2字节)
            can_id = motor_id

            # 确保所有参数都是整数
            cmd_byte = 0xAA
            pos_int = int(pos)
            spd_int = int(spd)
            cur_int = int(cur) & 0xFFFF

            # print(f"调试: motor_id={motor_id}, pos={pos_int}, spd={spd_int}, cur={cur_int}")
            data = struct.pack('<BHHH', cmd_byte, pos_int, spd_int, cur_int)

            return self.can_socket.send_message(can_id, data)

        except Exception as e:
            self.get_logger().error(f"电机{motor_id}控制失败: {e}")
            return False

    def _send_can_commands(self, motor_cmd_data):
        """发送CAN命令到硬件 (改进版本)"""
        if not self.can_socket or not self.can_socket.is_connected:
            return

        try:
            # 将电机命令数据转换为CAN消息并发送
            # 每3个数据为一组：位置、速度、电流限制
            for i in range(0, len(motor_cmd_data), 3):
                if i + 2 < len(motor_cmd_data):
                    motor_id = (i // 3) + 1  # 电机ID从1开始
                    position = motor_cmd_data[i]
                    speed = motor_cmd_data[i + 1]
                    current_limit = motor_cmd_data[i + 2]

                    # 使用混合控制发送命令 (对应C++的RyMotion_ServoMove_Mix)
                    self.servo_move_mix(motor_id, position, speed, current_limit)

        except Exception as e:
            self.get_logger().error(f"CAN命令发送失败: {e}")

    def process_received_can_message(self, can_id: int, data: bytes):
        """处理接收到的CAN消息 (类似C++的RyCanServoLibRcvMsg)"""
        if len(data) < 1:
            return

        try:
            cmd = data[0]

            # 根据C++版本，电机响应的CAN ID应该是 SERVO_BACK_ID(motor_id)
            # 通常是 motor_id + 256，但我们先简单处理
            if can_id >= 1 and can_id <= 16:
                motor_id = can_id - 1  # 转换为数组索引

                # 打印接收到的消息用于调试
                self.get_logger().info(f"收到电机{can_id}响应: 命令=0x{cmd:02X}, 数据={data.hex()}")

                if cmd in [0xA0, 0xA1, 0xA6, 0xA9, 0xAA]:
                    # 解析电机状态数据
                    if len(data) >= 8:
                        self.motor_status[motor_id] = data[1]
                        self.motor_positions[motor_id] = struct.unpack('<H', data[2:4])[0]
                        speed = struct.unpack('<h', data[4:6])[0]
                        current = struct.unpack('<h', data[6:8])[0]

                        # 处理负数
                        if speed > 2047:
                            speed -= 4096
                        if current > 2047:
                            current -= 4096

                        self.motor_currents[motor_id] = current

                        self.get_logger().info(f"电机{motor_id+1}状态: 位置={self.motor_positions[motor_id]}, 电流={current}, 状态=0x{self.motor_status[motor_id]:02X}")

                        # 更新状态消息
                        self.current_msg.motor_positions[motor_id] = self.motor_positions[motor_id]
                        self.current_msg.motor_currents[motor_id] = self.motor_currents[motor_id]
                        self.current_msg.motor_status[motor_id] = self.motor_status[motor_id]

            elif can_id >= 257 and can_id <= 271:  # SERVO_BACK_ID 范围
                motor_id = can_id - 257  # 转换为数组索引
                self.get_logger().info(f"收到电机{motor_id+1}回复: ID=0x{can_id:02X}, 命令=0x{cmd:02X}, 数据={data.hex()}")

        except Exception as e:
            self.get_logger().error(f"CAN消息处理错误: {e}")

    def command_callback(self, msg):
        """命令回调函数"""
        self.get_logger().info(f"收到命令: 模式={msg.mode}, LR={msg.lr}")

        # 直接处理命令，不检查变化 (类似C++版本的实时处理)
        self.current_cmd = msg
        self._process_command(msg)
    
    def _command_changed(self, new_cmd):
        """检查命令是否有变化"""
        try:
            # 安全的数组比较
            m_pos_changed = (len(new_cmd.m_pos) != len(self.current_cmd.m_pos) or
                           any(a != b for a, b in zip(new_cmd.m_pos, self.current_cmd.m_pos)))
            j_ang_changed = (len(new_cmd.j_ang) != len(self.current_cmd.j_ang) or
                           any(a != b for a, b in zip(new_cmd.j_ang, self.current_cmd.j_ang)))

            return (
                new_cmd.lr != self.current_cmd.lr or
                new_cmd.mode != self.current_cmd.mode or
                m_pos_changed or
                j_ang_changed
            )
        except Exception as e:
            # 如果比较失败，假设命令有变化
            self.get_logger().debug(f"命令比较错误: {e}")
            return True
    
    def _process_command(self, cmd):
        """处理命令"""
        if cmd.mode == 0:
            # 原始电机命令模式
            self._handle_raw_motor_command(cmd)
        elif cmd.mode == 1:
            # 关节角度命令模式
            self._handle_joint_angle_command(cmd)
        elif cmd.mode == 2:
            # 末端位置命令模式
            self._handle_end_position_command(cmd)
        else:
            self.get_logger().warn(f"未知命令模式: {cmd.mode}")
    
    def _handle_raw_motor_command(self, cmd):
        """处理原始电机命令"""
        # 第一次收到命令时清除电机故障
        if not self.fault_cleared:
            self.get_logger().info("首次收到命令，清除电机故障...")
            success = self.can_socket.send_message(0, struct.pack('B', 0xA1))
            if success:
                self.get_logger().info("电机故障清除完成")
                self.fault_cleared = True
            else:
                self.get_logger().warn("电机故障清除失败")
            import time
            time.sleep(0.01)  # 等待10ms

        self.get_logger().info(f"处理原始电机命令: 模式={cmd.mode}")

        # 直接发送电机命令到硬件
        for i in range(16):
            motor_id = i + 1  # 电机ID从1开始
            pos = cmd.m_pos[i]
            spd = cmd.m_spd[i]
            cur = cmd.m_curlimit[i]

            # 发送混合控制命令到硬件
            success = self.servo_move_mix(motor_id, pos, spd, cur)
            if not success:
                self.get_logger().warn(f"电机{motor_id}命令发送失败")

        # 发布电机命令消息 (用于其他节点监听)
        try:
            motor_cmd_msg = Int32MultiArray()
            motor_cmd_data = []

            for i in range(16):
                motor_cmd_data.extend([
                    int(i + 1),  # motor_id (1-based) - 确保是整数
                    int(cmd.m_pos[i]),      # 确保是整数
                    int(cmd.m_spd[i]),      # 确保是整数
                    int(cmd.m_curlimit[i])  # 确保是整数
                ])

            motor_cmd_msg.data = motor_cmd_data
            self.motor_cmd_pub.publish(motor_cmd_msg)
        except Exception as e:
            self.get_logger().error(f"发布电机命令消息失败: {e}")
    
    def _handle_joint_angle_command(self, cmd):
        """处理关节角度命令"""
        self.get_logger().debug("处理关节角度命令")
        
        # 将关节角度转换为电机位置
        motor_positions = joints_to_motors(cmd.j_ang, POLY_COEFF)
        
        # 发送电机命令
        motor_cmd_msg = Int32MultiArray()
        motor_cmd_data = []
        
        for i in range(16):
            motor_cmd_data.extend([
                i + 1,  # motor_id (1-based)
                motor_positions[i],
                cmd.m_spd[i],
                cmd.m_curlimit[i]
            ])
        
        motor_cmd_msg.data = motor_cmd_data
        self.motor_cmd_pub.publish(motor_cmd_msg)

        # 发送CAN消息到硬件
        self._send_can_commands(motor_cmd_data)
    
    def _handle_end_position_command(self, cmd):
        """处理末端位置命令"""
        self.get_logger().debug("处理末端位置命令")
        
        # 调用逆运动学服务
        if self.ik_client.service_is_ready():
            request = Rh16Ik.Request()
            request.lr = cmd.lr
            request.x_base = cmd.x_base
            request.y_base = cmd.y_base
            request.z_base = cmd.z_base
            request.roll_base = cmd.roll_base
            request.pitch_base = cmd.pitch_base
            request.yaw_base = cmd.yaw_base
            
            for i in range(5):
                request.x[i] = cmd.x[i]
                request.y[i] = cmd.y[i]
                request.z[i] = cmd.z[i]
                request.roll[i] = cmd.roll[i]
                request.pitch[i] = cmd.pitch[i]
                request.yaw[i] = cmd.yaw[i]
            
            # 异步调用服务
            future = self.ik_client.call_async(request)
            future.add_done_callback(
                lambda f: self._ik_response_callback(f, cmd)
            )
        else:
            self.get_logger().warn("逆运动学服务不可用")
    
    def _ik_response_callback(self, future, original_cmd):
        """逆运动学响应回调"""
        try:
            response = future.result()
            
            # 使用计算得到的关节角度
            joint_cmd = Rh16Cmd()
            joint_cmd.lr = original_cmd.lr
            joint_cmd.mode = 1  # 切换到关节角度模式
            joint_cmd.j_ang = list(response.j_ang)
            joint_cmd.m_spd = original_cmd.m_spd
            joint_cmd.m_curlimit = original_cmd.m_curlimit
            
            # 处理关节角度命令
            self._handle_joint_angle_command(joint_cmd)
            
        except Exception as e:
            self.get_logger().error(f"逆运动学服务调用失败: {e}")
    
    def motor_positions_callback(self, msg):
        """电机位置回调"""
        if len(msg.data) >= 16:
            self.motor_positions = msg.data[:16]
    
    def motor_currents_callback(self, msg):
        """电机电流回调"""
        if len(msg.data) >= 16:
            self.motor_currents = msg.data[:16]
    
    def motor_status_callback(self, msg):
        """电机状态回调"""
        if len(msg.data) >= 16:
            self.motor_status = msg.data[:16]
    
    def publish_status(self):
        """发布状态"""
        # 更新状态消息
        self.current_msg.header.stamp = self.get_clock().now().to_msg()
        self.current_msg.lr = self.current_cmd.lr
        
        # 更新电机数据
        self.current_msg.m_pos = self.motor_positions.copy()
        self.current_msg.m_cur = self.motor_currents.copy()
        self.current_msg.status = self.motor_status.copy()
        
        # 计算关节角度
        self.current_msg.j_ang = motors_to_joints(
            self.motor_positions, 
            POLY_COEFF, 
            self.current_cmd.lr
        )
        
        # 更新基坐标系信息
        self.current_msg.x_base = self.current_cmd.x_base
        self.current_msg.y_base = self.current_cmd.y_base
        self.current_msg.z_base = self.current_cmd.z_base
        self.current_msg.roll_base = self.current_cmd.roll_base
        self.current_msg.pitch_base = self.current_cmd.pitch_base
        self.current_msg.yaw_base = self.current_cmd.yaw_base
        
        # 调用正运动学服务计算末端位置
        if self.fk_client.service_is_ready():
            request = Rh16Fk.Request()
            request.lr = self.current_msg.lr
            request.j_ang = self.current_msg.j_ang
            request.x_base = self.current_msg.x_base
            request.y_base = self.current_msg.y_base
            request.z_base = self.current_msg.z_base
            request.roll_base = self.current_msg.roll_base
            request.pitch_base = self.current_msg.pitch_base
            request.yaw_base = self.current_msg.yaw_base
            
            # 异步调用服务
            future = self.fk_client.call_async(request)
            future.add_done_callback(self._fk_response_callback)
        
        # 发布状态
        self.status_pub.publish(self.current_msg)
    
    def _fk_response_callback(self, future):
        """正运动学响应回调"""
        try:
            response = future.result()
            
            # 更新末端位置信息
            self.current_msg.x = list(response.x)
            self.current_msg.y = list(response.y)
            self.current_msg.z = list(response.z)
            self.current_msg.w = list(response.w)
            self.current_msg.i = list(response.i)
            self.current_msg.j = list(response.j)
            self.current_msg.k = list(response.k)
            self.current_msg.roll = list(response.roll)
            self.current_msg.pitch = list(response.pitch)
            self.current_msg.yaw = list(response.yaw)
            
        except Exception as e:
            self.get_logger().debug(f"正运动学服务调用失败: {e}")



    def destroy_node(self):
        """节点销毁时的清理工作 (类似C++的清理)"""
        global thread_go, can_thread

        self.get_logger().info("正在关闭RH16控制节点...")

        # 停止线程 (类似C++的thread_go = 0)
        thread_go = False

        if can_thread and can_thread.is_alive():
            can_thread.join(timeout=1.0)
            self.get_logger().info("CAN接收和时间节拍线程已停止")

        # 不需要停止库线程

        # 关闭CAN连接 (类似C++的close_can_socket)
        if self.can_socket:
            self.can_socket.close()
            self.get_logger().info("CAN连接已关闭")

        super().destroy_node()


def main(args=None):
    global thread_go

    # ROS2 初始化 (类似C++的rclcpp::init)
    rclpy.init(args=args)

    # 打印程序参数 (类似C++的参数打印)
    print("Program Arguments:")
    if args:
        for i, arg in enumerate(args):
            print(f"argv[{i}]: {arg}")

    try:
        # 创建节点 (类似C++的std::make_shared<rh16_ctrl>)
        node = RH16CtrlNode()

        # 使用多线程执行器 (类似C++的MultiThreadedExecutor)
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        # 设置信号处理
        def signal_handler(signum, frame):
            global thread_go
            node.get_logger().info("收到中断信号，正在关闭...")
            thread_go = False
            executor.shutdown()

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        # 开始执行 (类似C++的executor.spin())
        executor.spin()

    except KeyboardInterrupt:
        print("收到键盘中断")
    except Exception as e:
        print(f"节点运行错误: {e}")
    finally:
        # 关闭ROS2 (类似C++的rclcpp::shutdown)
        thread_go = False
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
