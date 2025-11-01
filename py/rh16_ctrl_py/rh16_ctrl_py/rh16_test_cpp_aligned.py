#!/usr/bin/env python3
"""
RH16 Test Node - 完全对齐C++版本
与C++版本rh16_test.cpp完全一致的Python实现
"""

import rclpy
from rclpy.node import Node
import math

from rh16_cmd_py.msg import Rh16Cmd


class RH16Test(Node):
    """RH16测试节点 - 完全对齐C++版本"""

    def __init__(self, name: str = "rh16_test"):
        super().__init__(name)

        self.get_logger().info(f"hello {name}")

        # 声明参数 - 添加左右手选择
        self.declare_parameter('test_mode', 0)      # 测试模式 0,1,2
        self.declare_parameter('hand_type', 'left') # 左右手选择 'left' or 'right'
        self.declare_parameter('cmd_topic', 'ryhand_cmd')

        # 获取参数
        test_mode = self.get_parameter('test_mode').get_parameter_value().integer_value
        self.hand_type = self.get_parameter('hand_type').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value

        # 初始化变量 - 与C++版本完全一致
        self.rh16cmd = Rh16Cmd()
        self.rh16cmd.mode = test_mode  # 从参数获取模式
        self.rh16cmd.lr = (self.hand_type == 'right')  # False=左手(0), True=右手(1)
        self.tick = 0
        self.tspan_ms = 5  # 5ms定时器，与C++一致

        self.get_logger().info(f"测试参数: 模式={self.rh16cmd.mode}, 手部={'右手' if self.rh16cmd.lr else '左手'}")
        
        # 初始化电机参数 - 与C++完全一致
        # 直接创建长度为16的列表，避免动态添加导致的约束违反
        self.rh16cmd.m_pos = [4095] * 16
        self.rh16cmd.m_spd = [1000] * 16
        self.rh16cmd.m_curlimit = [80] * 16
        
        # 初始化关节角度数组
        self.rh16cmd.j_ang = [0.0] * 20
        
        # 创建发布器 - 命令 (与C++版本完全一致)
        self.ryhand_cmd_publisher = self.create_publisher(
            Rh16Cmd,
            self.cmd_topic,  # 与C++版本完全一致的话题名
            1  # 与C++版本一致的队列深度
        )
        
        # 创建定时器 - tspan_ms ms (与C++版本完全一致)
        self.timer = self.create_timer(
            self.tspan_ms / 1000.0,  # 转换为秒
            self.pub_cmd  # 与C++版本函数名一致
        )
    
    def rad_to_deg(self, rad: float) -> float:
        """弧度转角度 - 与C++版本完全一致"""
        return rad * 180.0 / math.pi
    
    def deg_to_rad(self, deg: float) -> float:
        """角度转弧度 - 与C++版本完全一致"""
        return deg * math.pi / 180.0
    
    def pub_cmd(self):
        """发布命令 - 与C++版本PubCmd()完全一致"""
        # 计算正弦和余弦值 - 与C++版本完全一致
        fs = math.sin(2 * math.pi * self.tick / 4000)
        fc = math.cos(2 * math.pi * self.tick / 4000)
        
        # 根据模式执行不同的控制 - 与C++版本完全一致
        if self.rh16cmd.mode == 0:
            # raw cmd - 模式0：原始命令模式
            # 生成幅值为4000的正弦波，周期为4s
            for i in range(16):
                # 与C++版本#else分支完全一致
                p1 = 1600 + 800*fs + 1000
                p2 = abs(1000 * fc) + 500

                # 大拇指电机特殊处理 - 只固定侧摆关节，保留弯曲功能
                if i == 0:  # 电机1是大拇指根部侧摆关节，固定不动
                    p1 = 0  # 侧摆位置设置为0
                    p2 = 0  # 侧摆速度设置为0
                # 电机2、3是大拇指弯曲关节，正常运动（不需要特殊处理）

                self.rh16cmd.m_pos[i] = int(p1)
                self.rh16cmd.m_spd[i] = int(p2)
        
        elif self.rh16cmd.mode == 1:
            # angle cmd - 模式1：关节角度命令模式
            for i in range(0, 20, 4):  # 每4个为一组，对应一个手指
                p1 = self.deg_to_rad(0 + 10 * fs)
                p2 = self.deg_to_rad(20 + 20 * fs)
                p3 = self.deg_to_rad(30 + 30 * fs)
                p4 = self.deg_to_rad(30 + 30 * fs)
                
                self.rh16cmd.j_ang[i + 0] = p1
                self.rh16cmd.j_ang[i + 1] = p2
                self.rh16cmd.j_ang[i + 2] = p3
                self.rh16cmd.j_ang[i + 3] = p4
        
        elif self.rh16cmd.mode == 2:
            # end pos cmd - 模式2：末端位置命令模式
            # 这部分在C++中是注释掉的，保持一致
            pass
        
        # 更新tick计数器 - 与C++版本完全一致
        self.tick = (self.tick + self.tspan_ms) % 100000
        
        # 发布消息 - 与C++版本完全一致
        self.ryhand_cmd_publisher.publish(self.rh16cmd)


def main(args=None):
    """主函数 - 与C++版本main()完全一致"""
    # ROS2 初始化
    rclpy.init(args=args)
    
    # 创建节点
    node = RH16Test("rh16_test")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("测试节点停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
