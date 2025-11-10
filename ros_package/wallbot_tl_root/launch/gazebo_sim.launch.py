import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import AppendEnvironmentVariable

from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'wallbot_tl_root'

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory(package_name),
                     'models'))

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': ['-r ', os.path.join(get_package_share_directory(package_name), 'world', 'wallbot_world.world')]   # optional: start paused
            }.items())
    
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'wallbot',
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
        set_env_vars_resources,
        spawn_entity,
        joint_traj_cont_spawner,
        joint_broad_spawner
    ])