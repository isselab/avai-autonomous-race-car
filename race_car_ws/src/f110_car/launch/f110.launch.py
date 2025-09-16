# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    f110_car_pkg_share = get_package_share_directory('f110_car')
    slam_toolbox_config = PathJoinSubstitution([f110_car_pkg_share, "mapper_params_online_async.yaml"])
    start_rviz = LaunchConfiguration("start_rviz", default=False)
    exploration_speed = LaunchConfiguration("exploration_speed", default=0.25)
    planning_speed = LaunchConfiguration("planning_speed", default=1.0)

    m2p_node = Node(
        package="f110_car",
        namespace="f110",
        executable="move_to_point",
        name="move_to_point",
        parameters=[{"use_sim_time": True, "max_speed": exploration_speed}]
    )
    exploration_node = Node(
        package="f110_car",
        namespace="f110",
        executable="exploration_node",
        name="exploration_node",
        parameters=[{'use_sim_time': True}],
    )
    global_planning_node = Node(
        package="f110_car",
        namespace="f110",
        executable="global_planning_node",
        name="global_planning_node",
        parameters=[{'use_sim_time': True, "planning_speed": planning_speed}],
    )
    exploration_vis_node = Node(
        package="f110_car",
        namespace="f110",
        executable="exploration_vis_node",
        name="exploration_vis_node",
        parameters=[{'use_sim_time': True}],
    )
    yolo_node = Node(
        package="test_package",
        namespace="f110",
        executable="yolo_node",
        name="yolo_node",
        parameters=[{'use_sim_time': True}],
    )
    semantic_mapping_node = Node(
        package="test_package",
        namespace="f110",
        executable="semantic_mapping_node",
        name="semantic_mapping_node",
        parameters=[{'use_sim_time': True}],
    )
    cone_marker_node = Node(
        package="test_package",
        namespace="f110",
        executable="cone_marker_node",
        name="cone_marker_node",
        parameters=[{'use_sim_time': True}],
    )
    semantic_grid_visualizer_node = Node(
        package="test_package",
        namespace="f110",
        executable="semantic_grid_visualizer_node",
        name="semantic_grid_visualizer_node",
        parameters=[{'use_sim_time': True}],
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([get_package_share_directory("slam_toolbox"),
                                                            "launch", "online_async_launch.py"])),
        launch_arguments={
            "use_sim_time": 'use_sim_time',
            "slam_params_file": slam_toolbox_config,
        }.items()

    )
    rviz = Node(
        package="rviz2",
        namespace="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", PathJoinSubstitution([f110_car_pkg_share, "rviz_config.rviz"])],
        condition=IfCondition(start_rviz)
    )
    transforms = GroupAction(
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='laser_camera_tf',
                output='screen',
                arguments=[
                    '0', '0', '0', '0.0', '0.0', '0.0',
                    'laser', 'camera_link'
                ],
                parameters=[{'use_sim_time': True}],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_scan_tf',
                output='screen',
                arguments=[
                    '0', '0', '0', '0.0', '0.0', '0.0',
                    'base_link', 'laser'
                ],
                parameters=[{'use_sim_time': True}],
            ),
        ])
    return LaunchDescription([
        transforms,
        exploration_node,
        exploration_vis_node,
        global_planning_node,
        yolo_node,
        cone_marker_node,
        semantic_mapping_node,
        semantic_grid_visualizer_node,
        slam_launch,
        rviz
    ])
