#!/usr/bin/env python3
"""
RH16 Kinematics Node
RH16运动学节点 - 负责正向和逆向运动学计算
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import time
import threading
from pathlib import Path
from typing import List, Tuple

from rh16_msg_py.srv import Rh16Fk, Rh16Ik
from .utils import (
    euler_to_rotation_matrix, 
    rotation_matrix_to_euler,
    rotation_matrix_to_quaternion,
    FINGERTIP_NAMES_LEFT,
    FINGERTIP_NAMES_RIGHT
)

# 尝试导入pinocchio
try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
    print("使用系统Pinocchio库")
except ImportError:
    try:
        # 尝试导入本地版本
        from .pinocchio_local import *
        import rh16_ctrl_py.pinocchio_local as pin
        PINOCCHIO_AVAILABLE = True
        print("使用本地Pinocchio库")
    except ImportError:
        PINOCCHIO_AVAILABLE = False
        print("警告: Pinocchio库不可用")


class RH16KinematicsNode(Node):
    """RH16运动学节点"""
    
    def __init__(self):
        super().__init__('rh16_kinematics_node')
        
        # 声明参数
        self.declare_parameter('urdf_path_left', '')
        self.declare_parameter('urdf_path_right', '')
        self.declare_parameter('use_pinocchio', True)
        
        # 获取参数
        urdf_path_left = self.get_parameter('urdf_path_left').get_parameter_value().string_value
        urdf_path_right = self.get_parameter('urdf_path_right').get_parameter_value().string_value
        self.use_pinocchio = self.get_parameter('use_pinocchio').get_parameter_value().bool_value
        
        self.get_logger().info('RH16运动学节点启动')
        
        # 设置URDF路径 - 与C++版本保持一致
        if not urdf_path_left:
            # 使用默认路径
            package_path = Path(__file__).parent.parent
            urdf_path_left = str(package_path / "urdf" / "rh1621hand_z.urdf")  # 对应ruihand16z.urdf

        if not urdf_path_right:
            package_path = Path(__file__).parent.parent
            urdf_path_right = str(package_path / "urdf" / "rh1621hand_y.urdf")  # 对应ruihand16y.urdf
        
        self.urdf_path_left = urdf_path_left
        self.urdf_path_right = urdf_path_right
        
        # 初始化Pinocchio模型
        self.model_l = None
        self.data_l = None
        self.model_r = None
        self.data_r = None
        
        if PINOCCHIO_AVAILABLE and self.use_pinocchio:
            self._load_pinocchio_models()
        else:
            self.get_logger().warn("Pinocchio不可用，运动学功能将受限")
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 服务
        self.fk_service = self.create_service(
            Rh16Fk,
            'rh16_forward_kinematics',
            self.forward_kinematics_callback
        )
        
        self.ik_service = self.create_service(
            Rh16Ik,
            'rh16_inverse_kinematics',
            self.inverse_kinematics_callback
        )
        
        self.get_logger().info("运动学服务已启动")
    
    def _load_pinocchio_models(self):
        """加载Pinocchio模型"""
        try:
            # 加载左手模型
            if Path(self.urdf_path_left).exists():
                self.model_l = pin.buildModelFromUrdf(self.urdf_path_left)
                self.data_l = self.model_l.createData()
                self.get_logger().info(f"左手模型加载成功: {self.urdf_path_left}")
            else:
                self.get_logger().warn(f"左手URDF文件不存在: {self.urdf_path_left}")
                # 使用默认模型
                self.model_l = pin.buildModelFromUrdf("dummy_path")
                self.data_l = self.model_l.createData()
            
            # 加载右手模型
            if Path(self.urdf_path_right).exists():
                self.model_r = pin.buildModelFromUrdf(self.urdf_path_right)
                self.data_r = self.model_r.createData()
                self.get_logger().info(f"右手模型加载成功: {self.urdf_path_right}")
            else:
                self.get_logger().warn(f"右手URDF文件不存在，使用左手模型")
                self.model_r = self.model_l
                self.data_r = self.data_l if self.model_l else None
            
            if self.model_l:
                self.get_logger().info(f"模型加载完成 - 关节数: {self.model_l.njoints}, 自由度: {self.model_l.nq}")
            
        except Exception as e:
            self.get_logger().error(f"Pinocchio模型加载失败: {e}")
            self.model_l = None
            self.data_l = None
            self.model_r = None
            self.data_r = None
    
    def forward_kinematics_callback(self, request, response):
        """正向运动学服务回调"""
        self.get_logger().debug("收到正向运动学请求")
        
        if not PINOCCHIO_AVAILABLE or not self.model_l:
            self.get_logger().error("Pinocchio不可用，无法执行正向运动学")
            return response
        
        try:
            # 选择模型
            if request.lr:
                model = self.model_r if self.model_r else self.model_l
                data = self.data_r if self.data_r else self.data_l
                fingertip_names = FINGERTIP_NAMES_RIGHT
            else:
                model = self.model_l
                data = self.data_l
                fingertip_names = FINGERTIP_NAMES_LEFT
            
            # 设置关节角度
            q = np.zeros(model.nq)
            for i in range(0, min(20, len(request.j_ang)), 4):
                finger_idx = i // 4
                if finger_idx * 5 + 3 < len(q):
                    q[finger_idx * 5 + 0] = request.j_ang[i + 0]
                    q[finger_idx * 5 + 1] = request.j_ang[i + 1]
                    q[finger_idx * 5 + 2] = request.j_ang[i + 2]
                    q[finger_idx * 5 + 3] = request.j_ang[i + 3]
            
            # 执行正向运动学
            pin.forwardKinematics(model, data, q)
            
            # 获取末端执行器位姿
            end_effector_poses = []
            for i in range(5):
                try:
                    joint_id = model.getJointId(fingertip_names[i])
                    end_effector_pose = data.oMi[joint_id]
                    end_effector_poses.append(end_effector_pose)
                except:
                    # 如果找不到关节，使用单位变换
                    end_effector_poses.append(pin.SE3.Identity())
            
            # 创建基坐标系变换
            translation = np.array([request.x_base, request.y_base, request.z_base])
            rotation_matrix = euler_to_rotation_matrix(request.roll_base, request.pitch_base, request.yaw_base)
            base_to_world = pin.SE3(rotation_matrix, translation)
            
            # 计算末端执行器相对于世界坐标系的位姿
            for i in range(5):
                pose = end_effector_poses[i]
                world_pose = base_to_world * pose
                
                response.x[i] = world_pose.translation[0]
                response.y[i] = world_pose.translation[1]
                response.z[i] = world_pose.translation[2]
                
                # 转换为四元数
                quat = rotation_matrix_to_quaternion(world_pose.rotation)
                response.w[i] = quat[3]  # w
                response.i[i] = quat[0]  # x
                response.j[i] = quat[1]  # y
                response.k[i] = quat[2]  # z
                
                # 转换为欧拉角
                euler = rotation_matrix_to_euler(world_pose.rotation)
                response.roll[i] = euler[0]
                response.pitch[i] = euler[1]
                response.yaw[i] = euler[2]
            
            self.get_logger().debug("正向运动学计算完成")
            
        except Exception as e:
            self.get_logger().error(f"正向运动学计算失败: {e}")
        
        return response
    
    def inverse_kinematics_callback(self, request, response):
        """逆向运动学服务回调"""
        self.get_logger().debug("收到逆向运动学请求")
        
        if not PINOCCHIO_AVAILABLE or not self.model_l:
            self.get_logger().error("Pinocchio不可用，无法执行逆向运动学")
            return response
        
        try:
            # 选择模型
            if request.lr:
                model = self.model_r if self.model_r else self.model_l
                data = self.data_r if self.data_r else self.data_l
                fingertip_names = FINGERTIP_NAMES_RIGHT
            else:
                model = self.model_l
                data = self.data_l
                fingertip_names = FINGERTIP_NAMES_LEFT
            
            # 执行逆向运动学
            result_q = self._solve_inverse_kinematics(
                model, data, fingertip_names, request
            )
            
            # 填充响应
            for i in range(min(20, len(result_q))):
                response.j_ang[i] = result_q[i]
            
            self.get_logger().debug("逆向运动学计算完成")
            
        except Exception as e:
            self.get_logger().error(f"逆向运动学计算失败: {e}")
        
        return response
    
    def _solve_inverse_kinematics(self, model, data, fingertip_names, request):
        """求解逆向运动学"""
        # 简化的逆运动学实现
        # 在实际应用中，这里应该实现完整的IK求解器
        
        q_init = pin.neutral(model)
        
        # 这里可以实现更复杂的IK算法
        # 目前返回中性配置
        return q_init


def main(args=None):
    rclpy.init(args=args)
    
    node = RH16KinematicsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
