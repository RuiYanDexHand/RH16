import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rh16_ctrl_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RH16 Team',
    maintainer_email='rh16@example.com',
    description='RH16 Hand Controller for Python - 16自由度手部控制器',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 核心节点
            'rh16_ctrl_node = rh16_ctrl_py.rh16_ctrl_node:main',
            'rh16_kinematics_node = rh16_ctrl_py.rh16_kinematics_node:main',
            'rh16_can_node = rh16_ctrl_py.rh16_can_node:main',
            'rh16_test_cpp_aligned = rh16_ctrl_py.rh16_test_cpp_aligned:main',

            # 简化的命令 - 与C++版本对应
            'rh_ctrl = rh16_ctrl_py.rh16_ctrl_node:main',      # 对应 ros2 run rh16_ctrl rh_ctrl
            'rh_test = rh16_ctrl_py.rh16_test_cpp_aligned:main', # 对应 ros2 run rh16_ctrl rh_test

            # 工具命令

        ],
    },
)
