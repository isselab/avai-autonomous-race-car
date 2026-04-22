# SPDX-License-Identifier: MIT
# Authors: Nicklas Osmers, Kevin Losing, Ahmad Al Shihabi, Sai Tarun Bhyri.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, 
                            IncludeLaunchDescription, SetLaunchConfiguration, TimerAction)
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    pkg_gazebo_f110 = get_package_share_directory('gazebo_f110')
    slam_toolbox_config = PathJoinSubstitution([pkg_gazebo_f110, "mapper_params_online_async.yaml"])
    
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    start_rviz = LaunchConfiguration("start_rviz", default=True)
    exploration_speed = LaunchConfiguration("exploration_speed", default=0.25)
    planning_speed = LaunchConfiguration("planning_speed", default=1.0)
    map_file = TextSubstitution(text="/home/tarunbhyri9/project_repo/avai-autonomous-race-car/race_car_ws/map_1772124162.yaml")
    
    wasd_node = Node(
            package="test_package",
            namespace="f110",
            executable="wasd_control_node",
            name="wasd_control",
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            )
    move_to_point = Node(
            package="f110_car",
            namespace="f110",
            executable="move_to_point",
            name="move_to_point",
            output='log',
            parameters=[{"use_sim_time": use_sim_time, "max_speed": exploration_speed}]
            )
    m2p_delayed = TimerAction(period=20.0, actions=[move_to_point])

    exploration_node = Node(
            package="f110_car",
            namespace="f110",
            executable="exploration_node",
            name="exploration_node",
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            )
    exploration_delayed = TimerAction(period=16.0, actions=[exploration_node])

    exploration_vis_node = Node(
            package="f110_car",
            namespace="f110",
            executable="exploration_vis_node",
            name="exploration_vis_node",
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            )

    global_planning_node = Node(
            package="f110_car",
            namespace="f110",
            executable="global_planning_node",
            name="global_planning_node",
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, "planning_speed": planning_speed}],
        )
    global_planning_delayed = TimerAction(period=18.0, actions=[global_planning_node])

    yolo_node_rgbd = Node(
            package="test_package",
            namespace="f110",
            executable="yolo_node_rgbd",
            name="yolo_node_rgbd",
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            )
    yolo_delayed = TimerAction(period=10.0, actions=[yolo_node_rgbd])

    semantic_mapping_node = Node(
            package="test_package",
            namespace="f110",
            executable="semantic_mapping_node",
            name="semantic_mapping_node",
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            )
    semantic_mapping_delayed = TimerAction(period=12.0, actions=[semantic_mapping_node],)
    
    cone_marker_node = Node(
            package="test_package",
            namespace="f110",
            executable="cone_marker_node",
            name="cone_marker_node",
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            )
    semantic_grid_visualizer_node = Node(
            package="test_package",
            namespace="f110",
            executable="semantic_grid_visualizer_node",
            name="semantic_grid_visualizer_node",
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            )
    
    
    ackermann_to_twist_node = Node(
            package="gazebo_f110",
            namespace="gazebo",
            executable="ackermann_to_twist",
            name="ackermann_to_twist",
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            )
    transform_node = Node(
        package="gazebo_f110",
        namespace="gazebo",
        executable="transform_pose",
        name="transform_pose",
        parameters=[{'use_sim_time': use_sim_time}],
    )
    rviz = Node(
            package="rviz2",
            namespace="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", PathJoinSubstitution([pkg_gazebo_f110, "rviz_config.rviz"])],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(start_rviz)
            )
    rviz_delayed = TimerAction(period=22.0, actions=[rviz])

    #slam_launch = IncludeLaunchDescription(
       #     PythonLaunchDescriptionSource(PathJoinSubstitution([get_package_share_directory("slam_toolbox"),
      #                                                          "launch", "online_async_launch.py"])),
      #      launch_arguments={
      #          "slam_params_file": slam_toolbox_config,
      #          "use_sim_time": "true"
      #          }.items()
     #       )
    #slam_launch_delayed = TimerAction(
    #period=5.0,
    #actions=[slam_launch],)

    localization_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([get_package_share_directory("slam_toolbox"), "launch", "localization_launch.py"])
    ),
    launch_arguments={
        "use_sim_time": "true",
        "map_file_name": map_file,
        "slam_params_file": slam_toolbox_config,
    }.items()
    )

    localization_launch_delayed = TimerAction(
    period=8.0,
    actions=[localization_launch],
    )


    ros_gz_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist", 
                "/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
                "/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",
                "/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                "/world/car_world/pose/info@geometry_msgs/msg/PoseArray@ignition.msgs.Pose_V",
                "/model/base_link/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
                "/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock",
                "/rgbd_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
                "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
                "/rgbd_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
                ],
            remappings=[
                ("/rgbd_camera/camera_info", "/camera/realsense2_camera/color/camera_info"),
                ("/rgbd_camera/image", "/camera/realsense2_camera/color/image_raw"),
                ("/rgbd_camera/depth_image", "/camera/realsense2_camera/depth/image_rect_raw"),
                ("/odom", "/odom"),
                ("/model/base_link/tf", "/tf"),
                ],
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            )

    gazebo_launch_group = GroupAction(
            actions = [
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
                       'on_exit_shutdown': 'true',
                       'use_sim_time': 'true',
                       'autostart': 'true'
                       }.items(),
                   )
               ]
            )
    #Kind of hacky way to make sure that this transform is the latest transform (after the car model transforms) and the camera->base_link coordinate transform works
    static_transform_publisher = TimerAction(
        period=5.0,
        actions=[Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_tf',
                    arguments=[
                        '0.12', '0', '0.155',
                        '-1.57', '0', '-1.57',
                        'base_link', 'camera_link'
                    ],
                    parameters=[{'use_sim_time': use_sim_time}],
                )]
    )
    transforms = GroupAction(
            actions = [
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='gpu_lidar_tf',
                    output='screen',
                    arguments=[
                        '0.12', '0', '0.055', '0.0', '0.0', '0.0',
                        'base_link', 'lidar_link'
                        ],
                    parameters=[{'use_sim_time': use_sim_time}],
                    )
                #Node(
                    #package='tf2_ros',
                    #executable='static_transform_publisher',
                    #name='map_scan_tf',
                    #output='screen',
                    #arguments=[
                    #    '0', '0', '0', '0.0', '0.0', '0.0',
                    #    'map', 'scan'
                    #   ],
                    #parameters=[{'use_sim_time': use_sim_time}],
                    #),
                #Node(
                    #package='tf2_ros',
                    #executable='static_transform_publisher',
                    #name='map_base_link',
                    #output='screen',
                    #arguments=[
                    #    '0', '0', '0', '0.0', '0.0', '0.0',
                    #    'map', 'odom'
                    #   ],
                    #parameters=[{'use_sim_time': use_sim_time}],
                    #),
                #Node(
                #    package='tf2_ros',
                #    executable='static_transform_publisher',
                #    name='odom_base_link',
                #    output='screen',
                #    arguments=[
                #        '0', '0', '0', '0.0', '0.0', '0.0',
                #        'odom', 'base_link'
                #       ],
                #    parameters=[{'use_sim_time': use_sim_time}],
                #    ),
                ])
    # robot_state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
    #         name='robot_state_publisher',
    #         output='screen',
    #         arguments=[PathJoinSubstitution([pkg_gazebo_f110, "model", "car", "f110_car.sdf"])])
    return LaunchDescription([
        gazebo_launch_group,
        m2p_delayed,
        exploration_delayed,
        #exploration_vis_node,
        global_planning_delayed,
        #wasd_node,
        yolo_delayed,
        #cone_marker_node,
        semantic_mapping_delayed,
        #semantic_grid_visualizer_node,
        transforms,
        transform_node,
        #ackermann_to_twist_node,
        ros_gz_bridge_node,
        localization_launch_delayed,
        rviz_delayed,
        #robot_state_publisher,
        static_transform_publisher
        ])
