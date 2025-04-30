from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'f110_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('*.yaml')),
        (os.path.join('share', package_name), glob('*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            "move_to_point = f110_car.m2p_node:main",
            "init_drive = f110_car.init_drive_node:main",
            "semantic_grid_vis = f110_car.semantic_grid_vis_node:main",
            "exploration_node = f110_car.exploration_node:main",
            "exploration_vis_node = f110_car.exploration_visualizer_node:main",
            "global_planning_node = f110_car.global_planning_node:main",
        ],
    },
)
