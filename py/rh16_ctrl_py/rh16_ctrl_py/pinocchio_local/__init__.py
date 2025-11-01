"""
Pinocchio Local Library - 本地Pinocchio库实现
用于替代系统安装的pinocchio库
"""

from .core import *
from .spatial import *
from .model import *
from .algorithms import *

__version__ = "1.0.0-local"
__author__ = "RH16 Team"

# 导出主要类和函数
__all__ = [
    'SE3',
    'Model', 
    'Data',
    'buildModelFromUrdf',
    'forwardKinematics',
    'neutral',
    'integrate',
    'computeJointJacobian',
    'log6',
    'randomConfiguration',
    'updateFramePlacements',
    'computeFrameJacobian',
]
