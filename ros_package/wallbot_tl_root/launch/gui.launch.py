#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = get_package_share_directory('wallbot_tl_root')
    gui_script = PathJoinSubstitution([pkg_share, '_gui.py'])
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', gui_script],
            output='screen'
        )
    ])
