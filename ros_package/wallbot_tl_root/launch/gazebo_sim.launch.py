import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'wallbot_tl_root'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': os.path.join(get_package_share_directory(package_name), 'world', 'wallbot_world.world')   # optional: start paused
            }.items())
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'wallbot',
                                   '-x', '0.0', '-y', '0.0', '-z', '3'])
    
    joint_broad_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],

            )

    joint_traj_cont_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller"],
            )


    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        joint_traj_cont_spawner,
        joint_broad_spawner
    ])