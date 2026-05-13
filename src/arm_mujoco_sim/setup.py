from setuptools import setup
import os
from glob import glob

package_name = 'arm_mujoco_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 安装Mujoco模型文件
        (os.path.join('share', package_name, 'mjcf'), glob('mjcf/*.xml')),
        # 【关键补充】安装meshes目录（STL模型）
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
    ],
    entry_points={
        'console_scripts': [
            'arm_sim_display_node = arm_mujoco_sim.arm_display:main',#节点名=包名，文件名
        ],
    },
)