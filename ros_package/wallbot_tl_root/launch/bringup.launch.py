#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ------- Paths -------
    pkg_share = get_package_share_directory('wallbot_tl_root')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'wallbot.urdf'])
    world_path = PathJoinSubstitution([pkg_share, 'world', 'wallbot_world.world'])

    # Absolute path to your GUI project root (has pyproject.toml)
    gui_root = os.path.expanduser('~/ros2_ws/src/Wallbug/src/plant_gui_app')
    icon_dir = os.path.expanduser('~/ros2_ws/src/Wallbug/src/icon')

    # ------- Robot Description (robot_state_publisher) -------
    robot_description = ParameterValue(
        Command(['cat ', urdf_path]),   # NOTE: space after 'cat'
        value_type=str
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # ------- Gazebo (Harmonic) -------
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path, '--render-engine', 'ogre2'],
        output='screen'
    )

    # ------- Spawn robot -------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'wallbot', '-topic', 'robot_description'],
        output='screen'
    )

    # ------- Bridge camera topics (RGB + NIR/NDVI) -------
    camera_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        # RGB
        '/world/wallbot_world/model/wallbot/link/top_left_socket/sensor/front_rgb_sensor/image'
        '@sensor_msgs/msg/Image[gz.msgs.Image',
        '/world/wallbot_world/model/wallbot/link/top_left_socket/sensor/front_rgb_sensor/camera_info'
        '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        # NIR (input to NDVI)
        '/world/wallbot_world/model/wallbot/link/top_left_socket/sensor/front_nir_sensor/image'
        '@sensor_msgs/msg/Image[gz.msgs.Image',
        '/world/wallbot_world/model/wallbot/link/top_left_socket/sensor/front_nir_sensor/camera_info'
        '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
    ],
    # These remaps create the short, GUI-friendly topics
    remappings=[
        # RGB
        ('/world/wallbot_world/model/wallbot/link/top_left_socket/sensor/front_rgb_sensor/image',
         '/front_rgb/image_raw'),
        ('/world/wallbot_world/model/wallbot/link/top_left_socket/sensor/front_rgb_sensor/camera_info',
         '/front_rgb/camera_info'),
        # NIR (NDVI input)
        ('/world/wallbot_world/model/wallbot/link/top_left_socket/sensor/front_nir_sensor/image',
         '/front_nir/image_raw'),
        ('/world/wallbot_world/model/wallbot/link/top_left_socket/sensor/front_nir_sensor/camera_info',
         '/front_nir/camera_info'),
    ],
    output='screen'
    )

    # ------- NDVI Node (converts NIR → NDVI) -------
    ndvi_node = Node(
        package='wallbot_tl_root',
        executable='ndvi_node',
        name='ndvi_node',
        output='screen'
    )

    # ------- GUI process (directly run app.py) -------
    gui_file = os.path.join(gui_root, 'plant_gui', 'app.py')
    gui_proc = ExecuteProcess(
        cmd=['python3', gui_file],
        cwd=gui_root,  # run from project root so relative imports & icons work
        output='screen',
        additional_env={
            # Ensure Python finds the "plant_gui" package
            'PYTHONPATH': gui_root + os.pathsep + os.environ.get('PYTHONPATH', ''),
            # Optional: make icons accessible to your app
            'PLANT_GUI_ICON_DIR': icon_dir,
        }
    )

    # ------- Delay until Gazebo fully loads -------
    delayed_actions = TimerAction(
        period=5.0,
        actions=[spawn_robot, camera_bridge, ndvi_node, gui_proc]
    )

    # ------- Launch sequence -------
    return LaunchDescription([
        rsp,
        gazebo,
        delayed_actions
    ])
