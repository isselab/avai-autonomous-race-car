from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gazebo_f110'

def collect_model_files(model_dir='model'):
    data_files = []
    for root, dirs, files in os.walk(model_dir):
        for file in files:
            file_path = os.path.join(root, file)
            rel_path = os.path.relpath(file_path, model_dir)
            target_dir = os.path.join('share', package_name, 'model', os.path.dirname(rel_path))
            data_files.append((target_dir, [file_path]))
    return data_files

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, "world"), glob('world/*.sdf')),
    (os.path.join('share', package_name), glob('*.yaml')),
    (os.path.join('share', package_name), glob('*.rviz')),
]

data_files += collect_model_files('model')

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            "ackermann_to_twist = gazebo_f110.ackermann_to_twist:main",
            "transform_pose = gazebo_f110.pose_transformer:main",
            "world_pose_to_odom = gazebo_f110.world_pose_to_odom:main",
            'odom_tf_node = gazebo_f110.odom_tf_node:main',
        ],
    },
)
