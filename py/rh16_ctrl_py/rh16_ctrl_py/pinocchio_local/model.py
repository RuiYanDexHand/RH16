"""
Robot model and data structures
"""

import numpy as np
from typing import List, Dict, Optional
from .spatial import SE3
import xml.etree.ElementTree as ET
from pathlib import Path


class Joint:
    """关节类"""
    def __init__(self, name: str, joint_type: str = "revolute"):
        self.name = name
        self.type = joint_type
        self.axis = np.array([0, 0, 1])  # 默认Z轴
        self.lower_limit = -np.pi
        self.upper_limit = np.pi
        self.parent_id = -1
        self.child_id = -1


class Link:
    """连杆类"""
    def __init__(self, name: str):
        self.name = name
        self.mass = 1.0
        self.inertia = np.eye(3)
        self.com = np.zeros(3)  # 质心位置


class Frame:
    """坐标系类"""
    def __init__(self, name: str, parent_joint_id: int, placement: SE3 = None):
        self.name = name
        self.parent_joint_id = parent_joint_id
        self.placement = placement if placement else SE3.Identity()


class Model:
    """机器人模型类"""
    
    def __init__(self):
        self.name = "robot"
        self.nq = 0  # 配置空间维度
        self.nv = 0  # 速度空间维度
        self.njoints = 0  # 关节数量
        
        # 关节和连杆
        self.joints: List[Joint] = []
        self.links: List[Link] = []
        self.frames: List[Frame] = []
        
        # 关节名称映射
        self.joint_names: List[str] = []
        self.link_names: List[str] = []
        self.frame_names: List[str] = []
        
        # 运动学树结构
        self.parents = []  # 父关节ID列表
        self.children = []  # 子关节ID列表
        
        # 关节限制
        self.lowerPositionLimit = np.array([])
        self.upperPositionLimit = np.array([])
        
        # 关节变换
        self.jointPlacements = []  # 关节相对于父关节的变换
        
        # 添加根关节
        self._add_root_joint()
    
    def _add_root_joint(self):
        """添加根关节"""
        root_joint = Joint("root", "fixed")
        root_link = Link("root")
        
        self.joints.append(root_joint)
        self.links.append(root_link)
        self.joint_names.append("root")
        self.link_names.append("root")
        
        self.parents.append(-1)
        self.children.append([])
        self.jointPlacements.append(SE3.Identity())
        
        self.njoints = 1
    
    def addJoint(self, parent_id: int, joint: Joint, placement: SE3, link: Link):
        """添加关节"""
        joint_id = self.njoints
        
        # 添加关节和连杆
        self.joints.append(joint)
        self.links.append(link)
        self.joint_names.append(joint.name)
        self.link_names.append(link.name)
        
        # 更新树结构
        self.parents.append(parent_id)
        self.children.append([])
        if parent_id >= 0:
            self.children[parent_id].append(joint_id)
        
        # 添加变换
        self.jointPlacements.append(placement)
        
        # 更新关节数量和自由度
        self.njoints += 1
        if joint.type == "revolute" or joint.type == "prismatic":
            self.nq += 1
            self.nv += 1
        
        return joint_id
    
    def getJointId(self, name: str) -> int:
        """根据名称获取关节ID"""
        try:
            return self.joint_names.index(name)
        except ValueError:
            # 如果找不到，尝试在frames中查找
            try:
                frame_idx = self.frame_names.index(name)
                return self.frames[frame_idx].parent_joint_id
            except ValueError:
                raise ValueError(f"Joint or frame '{name}' not found")
    
    def getFrameId(self, name: str) -> int:
        """根据名称获取frame ID"""
        try:
            return self.frame_names.index(name)
        except ValueError:
            raise ValueError(f"Frame '{name}' not found")
    
    def addFrame(self, frame: Frame):
        """添加frame"""
        self.frames.append(frame)
        self.frame_names.append(frame.name)
    
    def createData(self):
        """创建数据对象"""
        return Data(self)
    
    def updateLimits(self):
        """更新关节限制"""
        limits_lower = []
        limits_upper = []
        
        for joint in self.joints:
            if joint.type in ["revolute", "prismatic"]:
                limits_lower.append(joint.lower_limit)
                limits_upper.append(joint.upper_limit)
        
        self.lowerPositionLimit = np.array(limits_lower)
        self.upperPositionLimit = np.array(limits_upper)


class Data:
    """机器人数据类 - 存储运动学和动力学计算结果"""
    
    def __init__(self, model: Model):
        self.model = model
        
        # 关节变换 (相对于世界坐标系)
        self.oMi = [SE3.Identity() for _ in range(model.njoints)]
        
        # Frame变换 (相对于世界坐标系)
        self.oMf = [SE3.Identity() for _ in range(len(model.frames))]
        
        # 雅可比矩阵
        self.J = np.zeros((6, model.nv))
        
        # 关节速度和加速度
        self.v = [np.zeros(6) for _ in range(model.njoints)]
        self.a = [np.zeros(6) for _ in range(model.njoints)]
        
        # 其他动力学量
        self.tau = np.zeros(model.nv)
        self.nle = np.zeros(model.nv)  # 非线性效应
        self.g = np.zeros(model.nv)    # 重力项
        self.C = np.zeros((model.nv, model.nv))  # 科里奥利矩阵
        self.M = np.zeros((model.nv, model.nv))  # 质量矩阵


def buildModelFromUrdf(urdf_path: str, package_dirs: List[str] = None) -> Model:
    """从URDF文件构建模型"""
    urdf_file = Path(urdf_path)
    
    if not urdf_file.exists():
        print(f"警告: URDF文件不存在: {urdf_path}")
        # 返回一个默认的手部模型
        return _create_default_hand_model()
    
    try:
        # 解析URDF文件
        tree = ET.parse(urdf_file)
        root = tree.getroot()
        
        model = Model()
        model.name = root.get('name', 'robot')
        
        # 解析links
        links = {}
        for link_elem in root.findall('link'):
            link_name = link_elem.get('name')
            link = Link(link_name)
            links[link_name] = link
        
        # 解析joints
        joint_id_map = {'root': 0}  # 根关节ID为0
        
        for joint_elem in root.findall('joint'):
            joint_name = joint_elem.get('name')
            joint_type = joint_elem.get('type', 'revolute')
            
            # 创建关节
            joint = Joint(joint_name, joint_type)
            
            # 获取父子连杆
            parent_elem = joint_elem.find('parent')
            child_elem = joint_elem.find('child')
            
            if parent_elem is not None and child_elem is not None:
                parent_link = parent_elem.get('link')
                child_link = child_elem.get('link')
                
                # 获取父关节ID
                parent_id = joint_id_map.get(parent_link, 0)
                
                # 解析origin
                origin_elem = joint_elem.find('origin')
                placement = SE3.Identity()
                
                if origin_elem is not None:
                    xyz = origin_elem.get('xyz', '0 0 0').split()
                    rpy = origin_elem.get('rpy', '0 0 0').split()
                    
                    translation = np.array([float(x) for x in xyz])
                    roll, pitch, yaw = [float(x) for x in rpy]
                    
                    from .spatial import euler_to_rotation_matrix
                    rotation = euler_to_rotation_matrix(roll, pitch, yaw)
                    placement = SE3(rotation, translation)
                
                # 解析关节限制
                limit_elem = joint_elem.find('limit')
                if limit_elem is not None:
                    joint.lower_limit = float(limit_elem.get('lower', -np.pi))
                    joint.upper_limit = float(limit_elem.get('upper', np.pi))
                
                # 添加关节到模型
                if child_link in links:
                    joint_id = model.addJoint(parent_id, joint, placement, links[child_link])
                    joint_id_map[child_link] = joint_id
                    
                    # 添加frame
                    frame = Frame(joint_name, joint_id, SE3.Identity())
                    model.addFrame(frame)
        
        # 更新关节限制
        model.updateLimits()
        
        print(f"成功加载URDF模型: {model.name}, 关节数: {model.njoints}, 自由度: {model.nq}")
        return model
        
    except Exception as e:
        print(f"URDF解析失败: {e}")
        print("使用默认手部模型")
        return _create_default_hand_model()


def _create_default_hand_model() -> Model:
    """创建默认的手部模型"""
    model = Model()
    model.name = "default_hand"
    
    # 创建5个手指，每个手指5个关节
    finger_names = ["thumb", "index", "middle", "ring", "pinky"]
    joint_names_suffix = ["0", "1", "2", "3", "4"]
    
    for i, finger in enumerate(finger_names):
        parent_id = 0  # 都连接到根关节
        
        for j, suffix in enumerate(joint_names_suffix):
            joint_name = f"{finger}_{suffix}"
            link_name = f"{finger}_link_{suffix}"
            
            # 创建关节和连杆
            joint = Joint(joint_name, "revolute")
            joint.lower_limit = -np.pi/2
            joint.upper_limit = np.pi/2
            
            link = Link(link_name)
            
            # 设置关节位置 (简化的手指布局)
            x = (i - 2) * 0.02  # 手指间距
            y = j * 0.03        # 关节间距
            z = 0.0
            
            placement = SE3(np.eye(3), np.array([x, y, z]))
            
            # 添加到模型
            joint_id = model.addJoint(parent_id, joint, placement, link)
            parent_id = joint_id  # 下一个关节的父关节
            
            # 添加末端frame
            if j == len(joint_names_suffix) - 1:
                end_frame_name = f"f{finger[0]}{i+1}5"  # 如fz16, fz25等
                end_frame = Frame(end_frame_name, joint_id, SE3(np.eye(3), np.array([0, 0.02, 0])))
                model.addFrame(end_frame)
    
    # 更新关节限制
    model.updateLimits()
    
    print(f"创建默认手部模型: 关节数={model.njoints}, 自由度={model.nq}")
    return model
