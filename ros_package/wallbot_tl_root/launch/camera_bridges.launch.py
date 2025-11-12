# launch/camera_bridges.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Adjust world/model names if Gazebo scopes your topics differently
    bridges = [
        # RGB image + camera info
        # GZ topic names follow /world/<world>/model/<name>/link/<link>/sensor/<sensor>/image
        # Bridge matching with regex-ish selectors
        ("/world/.*/model/.*/link/front_rgb_link/sensor/front_rgb_sensor/image",
         "sensor_msgs/msg/Image"),
        ("/world/.*/model/.*/link/front_rgb_link/sensor/front_rgb_sensor/camera_info",
         "sensor_msgs/msg/CameraInfo"),

        # NIR (mono) image to feed NDVI node
        ("/world/.*/model/.*/link/front_ndvi_link/sensor/front_nir_sensor/image",
         "sensor_msgs/msg/Image"),
        ("/world/.*/model/.*/link/front_ndvi_link/sensor/front_nir_sensor/camera_info",
         "sensor_msgs/msg/CameraInfo"),
    ]

    args = []
    for gz_topic, ros_type in bridges:
        args += [f"{gz_topic}@{ros_type}[gz.msgs.Image"] if "Image" in ros_type else [f"{gz_topic}@{ros_type}[gz.msgs.CameraInfo"]

    # Simpler: one parameter_bridge with many args
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=args,
        output="screen"
    )

    return LaunchDescription([bridge])
