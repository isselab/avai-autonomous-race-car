# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
from setuptools import find_packages, setup

package_name = 'test_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'test_node = test_package.nodes.test_node:main',
            'wasd_control_node = test_package.nodes.wasd_control_node:main',
            'yolo_node = test_package.nodes.yolo_node:main',
            'lidar_node = test_package.nodes.lidar_node:main',
            'sensor_fusion_node = test_package.nodes.sensor_fusion_node:main',
            'mapping_node = test_package.nodes.mapping_node:main',
            'odom_tf_node = test_package.nodes.odom_tf_node:main',
            'semantic_mapping_node = test_package.nodes.semantic_mapping_node:main',
            'cone_marker_node = test_package.nodes.cone_marker_node:main',
            'semantic_grid_visualizer_node = test_package.nodes.semantic_grid_visualizer_node:main',
            'yolo_node_rgbd = test_package.nodes.yolo_node_rgbd:main'
        ],
    },
)
