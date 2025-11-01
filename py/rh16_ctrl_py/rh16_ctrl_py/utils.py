"""
Utility functions for RH16 hand controller
RH16手部控制器工具函数
"""

import math
import numpy as np
from typing import List


def rad_to_deg(rad: float) -> float:
    """弧度转角度"""
    return rad * 180.0 / math.pi


def deg_to_rad(deg: float) -> float:
    """角度转弧度"""
    return deg * math.pi / 180.0


def map_rad90_to_value(rad: float) -> int:
    """将弧度映射到0-4095的值 (0到π/2)"""
    if rad < 0:
        rad = 0
    elif rad > math.pi / 2:
        rad = math.pi / 2
    return int(4095 - (rad / (math.pi / 2)) * 4095)


def map_rad75_to_value(rad: float) -> int:
    """将弧度映射到0-4095的值 (0到75度)"""
    if rad < 0:
        rad = 0
    elif rad > math.pi * 75 / 180:
        rad = math.pi * 75 / 180
    return int(4095 - (rad / (math.pi * 75 / 180)) * 4095)


def map_rad_to_value_full_range(rad: float) -> int:
    """将弧度映射到-4095到4095的值 (-π/2到π/2)"""
    if rad < -math.pi / 2:
        rad = -math.pi / 2
    elif rad > math.pi / 2:
        rad = math.pi / 2
    return int((rad / (math.pi / 2)) * 4095)


def value_to_rad90(value: int) -> float:
    """将0-4095的值映射回弧度(0到π/2)"""
    if value < 0:
        value = 0
    elif value > 4095:
        value = 4095
    return (math.pi / 2) * (4095 - value) / 4095


def value_to_rad75(value: int) -> float:
    """将0-4095的值映射回弧度(0到75度)"""
    if value < 0:
        value = 0
    elif value > 4095:
        value = 4095
    return (math.pi * 75 / 180) * (4095 - value) / 4095


def value_to_rad_full_range(value: int) -> float:
    """将-4095到4095的值映射回弧度(-π/2到π/2)"""
    if value < -4095:
        value = -4095
    elif value > 4095:
        value = 4095
    return float(value) / 4095 * (math.pi / 2)


def evaluate_polynomial(coefficients: List[float], degree: int, x: float) -> float:
    """计算多项式的值"""
    result = 0.0
    for i in range(degree + 1):
        result += coefficients[degree - i] * (x ** i)
    return result


def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """欧拉角转旋转矩阵 (ZYX顺序)"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    
    R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    
    return R_z @ R_y @ R_x


def rotation_matrix_to_euler(R: np.ndarray) -> np.ndarray:
    """旋转矩阵转欧拉角 (ZYX顺序)"""
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    
    return np.array([x, y, z])


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """旋转矩阵转四元数 [x, y, z, w]"""
    trace = np.trace(R)
    
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    
    return np.array([x, y, z, w])


def joints_to_motors(j_ang: List[float], poly_coeff: np.ndarray) -> List[int]:
    """将关节角度转换为电机位置"""
    m_pos = [0] * 16
    
    for i in range(20):
        if i % 4 == 3:  # 每4个关节处理一次
            finger_idx = i // 4
            
            p1 = map_rad_to_value_full_range(j_ang[finger_idx * 4 + 0])
            p2 = map_rad90_to_value(j_ang[finger_idx * 4 + 1])
            p3 = map_rad75_to_value(j_ang[finger_idx * 4 + 2])
            
            # 限制p1范围
            p1 = max(-4095//4, min(4095//4, p1))
            
            # 计算电机位置
            p5 = p2 - p1 // 2
            p6 = p2 + p1 // 2
            
            # 限制范围
            p5 = max(0, min(4095, p5))
            p6 = max(0, min(4095, p6))
            p3 = max(0, min(4095, p3))
            
            # 设置电机位置
            motor_base = finger_idx * 3
            if motor_base + 2 < 16:
                m_pos[motor_base + 0] = p5
                m_pos[motor_base + 1] = p6
                m_pos[motor_base + 2] = p3
    
    return m_pos


def motors_to_joints(m_pos: List[int], poly_coeff: np.ndarray, lr: bool = False) -> List[float]:
    """将电机位置转换为关节角度"""
    j_ang = [0.0] * 20
    
    for i in range(16):
        if i % 3 == 2:  # 每三个电机处理一次
            finger_idx = i // 3
            
            # 处理左右手数据交换
            pos_1 = m_pos[i-2]
            pos_2 = m_pos[i-1]
            pos_3 = m_pos[i]
            
            if lr:  # 右手数据交换
                pos_1, pos_2 = pos_2, pos_1
            
            # 计算关节角度
            pcom = (pos_1 + pos_2) // 2
            perr = (pos_1 - pos_2)
            
            j_ang[finger_idx * 4 + 0] = value_to_rad_full_range(perr)
            j_ang[finger_idx * 4 + 1] = value_to_rad90(pcom)
            j_ang[finger_idx * 4 + 2] = value_to_rad75(pos_3)
            
            # 使用多项式计算第四个关节
            if finger_idx < len(poly_coeff):
                rad75_deg = rad_to_deg(value_to_rad75(pos_3))
                poly_result = evaluate_polynomial(poly_coeff[finger_idx].tolist(), 3, rad75_deg)
                j_ang[finger_idx * 4 + 3] = deg_to_rad(poly_result)
    
    return j_ang


# 多项式系数 - 与C++版本完全一致
POLY_COEFF = np.array([
    [-9.0538e-05, 0.0089736, 0.836899, 0.288791],  # 拇指
    [-9.0538e-05, 0.0089736, 0.836899, 0.288791],  # 食指
    [-7.0777e-05, 0.0061727, 0.882956, 0.207410],  # 中指
    [-9.0538e-05, 0.0089736, 0.836899, 0.288791],  # 无名指
    [-0.00011031, 0.011289, 0.801033, 0.363636],   # 小指
])


# 手指末端frame名称
FINGERTIP_NAMES_LEFT = ["fz1616", "fz1625", "fz1635", "fz1645", "fz1655"]
FINGERTIP_NAMES_RIGHT = ["fy1616", "fy1625", "fy1635", "fy1645", "fy1655"]
