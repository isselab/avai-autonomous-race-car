# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    pkg_gazebo_f110 = get_package_share_directory('gazebo_f110')

    ackermann_to_twist_node = Node(
        package="gazebo_f110",
        namespace="gazebo",
        executable="ackermann_to_twist",
        name="ackermann_to_twist"
    )
    world_pose_to_odom_node = Node(
        package="gazebo_f110",
        namespace="gazebo",
        executable="world_pose_to_odom",
        name="world_pose_to_odom"
    )
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
            "/world/car_world/pose/info@geometry_msgs/msg/PoseArray@ignition.msgs.Pose_V",
        ],
        remappings=[
        ],
        output='screen'
    )

    gazebo_launch_group = GroupAction(
        actions=[
            DeclareLaunchArgument(
                'world',
                default_value='plane',
                choices=['plane', 'circle'],
                description='World to load into Gazebo'
            ),
            SetLaunchConfiguration(name='world_file',
                                   value=[LaunchConfiguration('world'),
                                          TextSubstitution(text='.sdf')]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch_path),
                launch_arguments={
                    'ign_args': [PathJoinSubstitution([pkg_gazebo_f110, "world", LaunchConfiguration('world_file')])],
                    'on_exit_shutdown': 'True'
                }.items(),
            )
        ]
    )
    return LaunchDescription([
        gazebo_launch_group,
        world_pose_to_odom_node,
        ackermann_to_twist_node,
        ros_gz_bridge_node,
    ])
