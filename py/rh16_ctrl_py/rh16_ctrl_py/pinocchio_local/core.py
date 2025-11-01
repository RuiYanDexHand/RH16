"""
Core pinocchio functionality
"""

# 重新导出所有主要功能
from .spatial import (
    SE3, 
    log6, 
    exp6,
    euler_to_rotation_matrix,
    rotation_matrix_to_euler,
    rotation_matrix_to_quaternion,
    quaternion_to_rotation_matrix
)

from .model import (
    Model,
    Data, 
    Joint,
    Link,
    Frame,
    buildModelFromUrdf
)

from .algorithms import (
    forwardKinematics,
    updateFramePlacements,
    computeJointJacobian,
    computeFrameJacobian,
    neutral,
    randomConfiguration,
    integrate,
    difference,
    interpolate,
    distance,
    normalize,
    isSatisfied,
    saturate,
    computeAllTerms,
    rnea,
    crba,
    nonLinearEffects
)

# 版本信息
__version__ = "1.0.0-local"

# 一些常用的常量
import numpy as np

# 重力加速度
GRAVITY = np.array([0, 0, -9.81])

# 一些便利函数
def buildModelsFromUrdf(urdf_path: str, mesh_dir: str = None, root_joint=None):
    """
    从URDF构建模型 (兼容原始pinocchio接口)
    返回 (model, collision_model, visual_model)
    """
    model = buildModelFromUrdf(urdf_path)
    
    # 简化版本，不处理碰撞和视觉模型
    collision_model = None
    visual_model = None
    
    return model, collision_model, visual_model


def createDatas(model, collision_model=None, visual_model=None):
    """
    创建数据对象 (兼容原始pinocchio接口)
    """
    data = model.createData()
    collision_data = None
    visual_data = None
    
    return data, collision_data, visual_data


def updateGeometryPlacements(model, data, collision_model=None, collision_data=None):
    """
    更新几何体位姿 (兼容原始pinocchio接口)
    """
    # 简化版本，不做任何操作
    pass


# RPY (Roll-Pitch-Yaw) 相关函数
class rpy:
    @staticmethod
    def matrixToRpy(R):
        """旋转矩阵转RPY角"""
        return rotation_matrix_to_euler(R)
    
    @staticmethod
    def rpyToMatrix(rpy_angles):
        """RPY角转旋转矩阵"""
        return euler_to_rotation_matrix(rpy_angles[0], rpy_angles[1], rpy_angles[2])


# 一些数学工具
def skew(v):
    """向量的反对称矩阵"""
    from .spatial import skew_symmetric
    return skew_symmetric(v)


def exp3(omega):
    """so(3)的指数映射"""
    theta = np.linalg.norm(omega)
    if theta < 1e-6:
        return np.eye(3)
    
    omega_hat = skew(omega)
    return np.eye(3) + np.sin(theta)/theta * omega_hat + (1-np.cos(theta))/(theta**2) * omega_hat @ omega_hat


def log3(R):
    """SO(3)的对数映射"""
    trace_R = np.trace(R)
    
    if abs(trace_R - 3) < 1e-6:
        return np.zeros(3)
    
    theta = np.arccos((trace_R - 1) / 2)
    if abs(theta) < 1e-6:
        return np.zeros(3)
    
    omega_hat = theta / (2 * np.sin(theta)) * (R - R.T)
    return np.array([omega_hat[2, 1], omega_hat[0, 2], omega_hat[1, 0]])


# 关节模型类 (用于兼容性)
class JointModelFreeFlyer:
    """自由飞行关节模型"""
    pass


class JointModelRevolute:
    """旋转关节模型"""
    pass


class JointModelPrismatic:
    """平移关节模型"""
    pass


class JointModelFixed:
    """固定关节模型"""
    pass


# 一些实用工具函数
def isNormalized(q, model=None, tolerance=1e-6):
    """检查配置是否已归一化"""
    if model is None:
        return True
    return isSatisfied(model, q, tolerance)


def squaredDistance(model, q1, q2):
    """配置的平方距离"""
    diff = difference(model, q1, q2)
    return np.dot(diff, diff)


def weightedDistance(model, q1, q2, weights):
    """加权距离"""
    diff = difference(model, q1, q2)
    return np.sqrt(np.dot(diff * weights, diff))


# 一些常用的数值方法
def finiteDifferenceIncrement():
    """有限差分增量"""
    return 1e-8


def computeJacobianTimeVariation(model, data, q, v):
    """计算雅可比矩阵的时间变化率 (简化实现)"""
    return np.zeros((6, model.nv))


# 算法选项类
class ReferenceConfiguration:
    """参考配置"""
    def __init__(self, q_ref):
        self.q_ref = q_ref


class IKSolverOptions:
    """逆运动学求解器选项"""
    def __init__(self):
        self.max_iterations = 100
        self.tolerance = 1e-6
        self.damping = 1e-6
        self.step_size = 0.1


# 一些预定义的配置
def getRandomConfiguration(model):
    """获取随机配置"""
    return randomConfiguration(model)


def getNeutralConfiguration(model):
    """获取中性配置"""
    return neutral(model)


# 简化的可视化接口 (占位符)
class GepettoVisualizer:
    """Gepetto可视化器 (占位符)"""
    def __init__(self, model, collision_model=None, visual_model=None):
        self.model = model
        print("注意: 这是简化版可视化器，不提供实际可视化功能")
    
    def initViewer(self, viewer=None):
        print("可视化器初始化 (模拟)")
    
    def loadViewerModel(self, rootNodeName="pinocchio"):
        print(f"加载可视化模型: {rootNodeName} (模拟)")
    
    def display(self, q):
        print(f"显示配置: {q[:min(5, len(q))]}... (模拟)")


# 导出所有主要符号
__all__ = [
    # 空间数学
    'SE3', 'log6', 'exp6', 'skew', 'exp3', 'log3',
    'euler_to_rotation_matrix', 'rotation_matrix_to_euler',
    'rotation_matrix_to_quaternion', 'quaternion_to_rotation_matrix',
    
    # 模型和数据
    'Model', 'Data', 'Joint', 'Link', 'Frame',
    'buildModelFromUrdf', 'buildModelsFromUrdf', 'createDatas',
    
    # 算法
    'forwardKinematics', 'updateFramePlacements', 'updateGeometryPlacements',
    'computeJointJacobian', 'computeFrameJacobian',
    'neutral', 'randomConfiguration', 'integrate',
    'difference', 'interpolate', 'distance', 'squaredDistance', 'weightedDistance',
    'normalize', 'isSatisfied', 'saturate',
    'computeAllTerms', 'rnea', 'crba', 'nonLinearEffects',
    
    # 关节模型
    'JointModelFreeFlyer', 'JointModelRevolute', 'JointModelPrismatic', 'JointModelFixed',
    
    # 工具
    'rpy', 'GRAVITY', 'finiteDifferenceIncrement',
    'isNormalized', 'getRandomConfiguration', 'getNeutralConfiguration',
    
    # 可视化
    'GepettoVisualizer',
    
    # 选项类
    'ReferenceConfiguration', 'IKSolverOptions',
]
