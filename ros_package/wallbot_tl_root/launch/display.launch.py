#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("wallbot_tl_root"),
            "yaml",
            "ros_controller.yaml",
        ]
    )

    model_arg = LaunchConfiguration('model')
    gui_arg = LaunchConfiguration('gui')
    rviz_arg = LaunchConfiguration('rvizconfig')

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', model_arg])
        }]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[position_goals],
        output="screen",
    )

    # Spawn the joint_state_broadcaster after ros2_control_node is up
    joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                    output="screen",
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
                    output="screen",
                )
            ]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='/home/cajwill/ros2_ws/src/wallbot_tl_root/urdf/wallbot.urdf'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rvizconfig', default_value='/home/cajwill/ros2_ws/src/wallbot_tl_root/rviz/urdf.rviz'),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui_arg),
            name='joint_state_publisher'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(gui_arg),
            name='joint_state_publisher'
        ),
        robot_state_pub,
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_arg],
            name='rviz',
            output='screen'
        ),
        ros2_control_node,
        joint_state_broadcaster_spawner
    ])



