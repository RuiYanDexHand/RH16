"""
Spatial mathematics for robotics - SE3, SO3 等空间变换
"""

import numpy as np
from typing import Union, Tuple


class SE3:
    """SE(3) 特殊欧几里得群 - 3D刚体变换"""
    
    def __init__(self, rotation: np.ndarray = None, translation: np.ndarray = None):
        """
        初始化SE3变换
        Args:
            rotation: 3x3旋转矩阵
            translation: 3x1平移向量
        """
        if rotation is None:
            self.rotation = np.eye(3)
        else:
            self.rotation = np.array(rotation, dtype=float)
            
        if translation is None:
            self.translation = np.zeros(3)
        else:
            self.translation = np.array(translation, dtype=float).flatten()
    
    @staticmethod
    def Identity():
        """返回单位变换"""
        return SE3()
    
    @staticmethod
    def Random():
        """返回随机变换"""
        # 随机旋转矩阵
        angles = np.random.uniform(-np.pi, np.pi, 3)
        R = euler_to_rotation_matrix(angles[0], angles[1], angles[2])
        
        # 随机平移
        t = np.random.uniform(-1, 1, 3)
        
        return SE3(R, t)
    
    def __mul__(self, other):
        """SE3变换的复合"""
        if isinstance(other, SE3):
            new_rotation = self.rotation @ other.rotation
            new_translation = self.rotation @ other.translation + self.translation
            return SE3(new_rotation, new_translation)
        else:
            raise TypeError("SE3只能与SE3相乘")
    
    def inverse(self):
        """返回逆变换"""
        R_inv = self.rotation.T
        t_inv = -R_inv @ self.translation
        return SE3(R_inv, t_inv)
    
    def actInv(self, other):
        """作用逆变换 self^{-1} * other"""
        return self.inverse() * other
    
    def act(self, point: np.ndarray):
        """作用于点"""
        if point.shape == (3,):
            return self.rotation @ point + self.translation
        elif point.shape == (4,):
            # 齐次坐标
            result = np.zeros(4)
            result[:3] = self.rotation @ point[:3] + self.translation
            result[3] = point[3]
            return result
        else:
            raise ValueError("点必须是3D或齐次坐标")
    
    def matrix(self):
        """返回4x4齐次变换矩阵"""
        T = np.eye(4)
        T[:3, :3] = self.rotation
        T[:3, 3] = self.translation
        return T
    
    def copy(self):
        """返回副本"""
        return SE3(self.rotation.copy(), self.translation.copy())
    
    def __str__(self):
        return f"SE3(\nR=\n{self.rotation}\nt={self.translation}\n)"


class Log6Result:
    """log6函数的结果"""
    def __init__(self, vector: np.ndarray):
        self.vector = vector.flatten()
    
    def toVector(self):
        return self.vector


def log6(se3: SE3) -> Log6Result:
    """SE3的对数映射到se3李代数"""
    R = se3.rotation
    t = se3.translation
    
    # 计算旋转的轴角表示
    trace_R = np.trace(R)
    
    if abs(trace_R - 3) < 1e-6:
        # 接近单位矩阵
        omega = np.zeros(3)
    else:
        theta = np.arccos((trace_R - 1) / 2)
        if abs(theta) < 1e-6:
            omega = np.zeros(3)
        else:
            omega_hat = theta / (2 * np.sin(theta)) * (R - R.T)
            omega = np.array([omega_hat[2, 1], omega_hat[0, 2], omega_hat[1, 0]])
    
    # 简化的平移部分处理
    rho = t
    
    # 组合成6维向量 [omega, rho]
    log_vector = np.concatenate([omega, rho])
    
    return Log6Result(log_vector)


def exp6(xi: np.ndarray) -> SE3:
    """se3李代数的指数映射到SE3"""
    omega = xi[:3]
    rho = xi[3:6]
    
    theta = np.linalg.norm(omega)
    
    if theta < 1e-6:
        # 接近零，使用泰勒展开
        R = np.eye(3)
        V = np.eye(3)
    else:
        # 罗德里格斯公式
        omega_hat = skew_symmetric(omega)
        R = np.eye(3) + np.sin(theta)/theta * omega_hat + (1-np.cos(theta))/(theta**2) * omega_hat @ omega_hat
        
        # V矩阵
        V = np.eye(3) + (1-np.cos(theta))/(theta**2) * omega_hat + (theta-np.sin(theta))/(theta**3) * omega_hat @ omega_hat
    
    t = V @ rho
    
    return SE3(R, t)


def skew_symmetric(v: np.ndarray) -> np.ndarray:
    """向量的反对称矩阵"""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


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


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """四元数转旋转矩阵 [x, y, z, w]"""
    x, y, z, w = q
    
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])
    
    return R
