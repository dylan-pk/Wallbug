# spawn_wallbot.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_share = get_package_share_directory('wallbot_bot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'Wallbot_V3_Robot.urdf')
    world_file = ''  # optional: path to a world file

    # Launch Gazebo
    gz_cmd = ['gazebo', '--verbose']
    if world_file:
        gz_cmd.append(world_file)
    # ensure factory plugin loaded
    gz_cmd += ['-s', 'libgazebo_ros_factory.so']

    spawn_cmd = [
      'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
      '-file', urdf_file,
      '-entity', 'wallbot_v3',
      '-x', '0', '-y', '0', '-z', '3'
    ]

    return LaunchDescription([
      ExecuteProcess(cmd=gz_cmd, output='screen'),
      ExecuteProcess(cmd=spawn_cmd, output='screen'),
    ])
