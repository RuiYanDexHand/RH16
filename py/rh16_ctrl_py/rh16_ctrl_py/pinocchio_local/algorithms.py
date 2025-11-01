"""
Robotics algorithms - kinematics, dynamics, etc.
"""

import numpy as np
from typing import List, Optional
from .model import Model, Data
from .spatial import SE3, log6, exp6, skew_symmetric


def forwardKinematics(model: Model, data: Data, q: np.ndarray):
    """正向运动学计算"""
    if len(q) != model.nq:
        print(f"警告: 关节角度维度({len(q)})与模型自由度({model.nq})不匹配")
        # 调整q的大小
        q_adjusted = np.zeros(model.nq)
        min_len = min(len(q), model.nq)
        q_adjusted[:min_len] = q[:min_len]
        q = q_adjusted
    
    # 初始化根关节变换
    data.oMi[0] = SE3.Identity()
    
    # 遍历所有关节
    q_idx = 0
    for joint_id in range(1, model.njoints):
        parent_id = model.parents[joint_id]
        joint = model.joints[joint_id]
        
        # 获取关节相对变换
        joint_placement = model.jointPlacements[joint_id]
        
        # 计算关节变换
        if joint.type == "revolute":
            if q_idx < len(q):
                angle = q[q_idx]
                q_idx += 1
            else:
                angle = 0.0
            
            # 绕Z轴旋转
            cos_a, sin_a = np.cos(angle), np.sin(angle)
            joint_rotation = np.array([
                [cos_a, -sin_a, 0],
                [sin_a,  cos_a, 0],
                [0,      0,     1]
            ])
            joint_transform = SE3(joint_rotation, np.zeros(3))
            
        elif joint.type == "prismatic":
            if q_idx < len(q):
                displacement = q[q_idx]
                q_idx += 1
            else:
                displacement = 0.0
            
            # 沿Z轴平移
            joint_transform = SE3(np.eye(3), np.array([0, 0, displacement]))
            
        else:  # fixed joint
            joint_transform = SE3.Identity()
        
        # 计算世界坐标系下的变换
        if parent_id >= 0:
            data.oMi[joint_id] = data.oMi[parent_id] * joint_placement * joint_transform
        else:
            data.oMi[joint_id] = joint_placement * joint_transform
    
    # 更新frame变换
    updateFramePlacements(model, data)


def updateFramePlacements(model: Model, data: Data):
    """更新frame位姿"""
    for i, frame in enumerate(model.frames):
        parent_joint_id = frame.parent_joint_id
        if parent_joint_id < len(data.oMi):
            data.oMf[i] = data.oMi[parent_joint_id] * frame.placement
        else:
            data.oMf[i] = frame.placement


def computeJointJacobian(model: Model, data: Data, q: np.ndarray, joint_id: int, J: np.ndarray):
    """计算关节雅可比矩阵"""
    if joint_id >= len(data.oMi):
        print(f"警告: 关节ID {joint_id} 超出范围")
        return
    
    # 获取目标关节的位姿
    target_pose = data.oMi[joint_id]
    target_pos = target_pose.translation
    
    # 初始化雅可比矩阵
    J.fill(0.0)
    
    # 遍历从根到目标关节的路径
    current_joint = joint_id
    q_idx = model.nq - 1
    
    while current_joint > 0 and q_idx >= 0:
        parent_id = model.parents[current_joint]
        joint = model.joints[current_joint]
        
        if joint.type == "revolute":
            # 获取关节位姿
            joint_pose = data.oMi[current_joint]
            joint_pos = joint_pose.translation
            joint_axis = joint_pose.rotation @ np.array([0, 0, 1])  # Z轴
            
            # 计算雅可比列
            pos_diff = target_pos - joint_pos
            
            # 角速度部分
            if q_idx < J.shape[1]:
                J[0:3, q_idx] = joint_axis
                J[3:6, q_idx] = np.cross(joint_axis, pos_diff)
            
            q_idx -= 1
        
        elif joint.type == "prismatic":
            # 平移关节
            joint_pose = data.oMi[current_joint]
            joint_axis = joint_pose.rotation @ np.array([0, 0, 1])  # Z轴
            
            if q_idx < J.shape[1]:
                J[0:3, q_idx] = np.zeros(3)
                J[3:6, q_idx] = joint_axis
            
            q_idx -= 1
        
        current_joint = parent_id


def computeFrameJacobian(model: Model, data: Data, q: np.ndarray, frame_id: int, J: np.ndarray = None):
    """计算frame雅可比矩阵"""
    if frame_id >= len(model.frames):
        print(f"警告: Frame ID {frame_id} 超出范围")
        return np.zeros((6, model.nv))
    
    frame = model.frames[frame_id]
    parent_joint_id = frame.parent_joint_id
    
    if J is None:
        J = np.zeros((6, model.nv))
    
    # 计算父关节的雅可比矩阵
    computeJointJacobian(model, data, q, parent_joint_id, J)
    
    return J


def neutral(model: Model) -> np.ndarray:
    """返回中性配置 (所有关节角度为0)"""
    return np.zeros(model.nq)


def randomConfiguration(model: Model) -> np.ndarray:
    """返回随机配置"""
    q = np.zeros(model.nq)
    
    for i in range(model.nq):
        if i < len(model.lowerPositionLimit) and i < len(model.upperPositionLimit):
            lower = model.lowerPositionLimit[i]
            upper = model.upperPositionLimit[i]
            q[i] = np.random.uniform(lower, upper)
        else:
            q[i] = np.random.uniform(-np.pi, np.pi)
    
    return q


def integrate(model: Model, q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """积分 q + v (简化版本)"""
    if len(v) != model.nv:
        print(f"警告: 速度维度({len(v)})与模型速度空间({model.nv})不匹配")
        v_adjusted = np.zeros(model.nv)
        min_len = min(len(v), model.nv)
        v_adjusted[:min_len] = v[:min_len]
        v = v_adjusted
    
    if len(q) != model.nq:
        print(f"警告: 配置维度({len(q)})与模型配置空间({model.nq})不匹配")
        q_adjusted = np.zeros(model.nq)
        min_len = min(len(q), model.nq)
        q_adjusted[:min_len] = q[:min_len]
        q = q_adjusted
    
    # 简单的欧拉积分
    return q + v


def difference(model: Model, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """计算配置差 q2 - q1"""
    return q2 - q1


def interpolate(model: Model, q1: np.ndarray, q2: np.ndarray, alpha: float) -> np.ndarray:
    """配置插值"""
    return (1 - alpha) * q1 + alpha * q2


def distance(model: Model, q1: np.ndarray, q2: np.ndarray) -> float:
    """配置距离"""
    diff = difference(model, q1, q2)
    return np.linalg.norm(diff)


def normalize(model: Model, q: np.ndarray) -> np.ndarray:
    """归一化配置 (将角度限制在[-pi, pi])"""
    q_norm = q.copy()
    
    for i in range(len(q_norm)):
        # 将角度归一化到[-pi, pi]
        while q_norm[i] > np.pi:
            q_norm[i] -= 2 * np.pi
        while q_norm[i] < -np.pi:
            q_norm[i] += 2 * np.pi
    
    return q_norm


def isSatisfied(model: Model, q: np.ndarray, tolerance: float = 1e-6) -> bool:
    """检查配置是否满足关节限制"""
    for i in range(min(len(q), len(model.lowerPositionLimit), len(model.upperPositionLimit))):
        if q[i] < model.lowerPositionLimit[i] - tolerance:
            return False
        if q[i] > model.upperPositionLimit[i] + tolerance:
            return False
    
    return True


def saturate(model: Model, q: np.ndarray) -> np.ndarray:
    """将配置限制在关节限制范围内"""
    q_sat = q.copy()
    
    for i in range(min(len(q_sat), len(model.lowerPositionLimit), len(model.upperPositionLimit))):
        q_sat[i] = np.clip(q_sat[i], model.lowerPositionLimit[i], model.upperPositionLimit[i])
    
    return q_sat


# 动力学相关函数 (简化实现)

def computeAllTerms(model: Model, data: Data, q: np.ndarray, v: np.ndarray):
    """计算所有动力学项"""
    # 简化实现，实际应该计算质量矩阵、科里奥利力等
    data.M.fill(0.0)
    np.fill_diagonal(data.M, 1.0)  # 单位质量矩阵
    
    data.nle.fill(0.0)  # 非线性效应
    data.g.fill(0.0)    # 重力项


def rnea(model: Model, data: Data, q: np.ndarray, v: np.ndarray, a: np.ndarray) -> np.ndarray:
    """递归牛顿-欧拉算法 (简化实现)"""
    # 返回零力矩
    return np.zeros(model.nv)


def crba(model: Model, data: Data, q: np.ndarray) -> np.ndarray:
    """复合刚体算法 - 计算质量矩阵 (简化实现)"""
    data.M.fill(0.0)
    np.fill_diagonal(data.M, 1.0)  # 单位质量矩阵
    return data.M


def nonLinearEffects(model: Model, data: Data, q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """计算非线性效应 (科里奥利力 + 重力) (简化实现)"""
    return np.zeros(model.nv)
