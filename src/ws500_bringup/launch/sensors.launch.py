"""Sobe a camera Orbbec Gemini E (RGB+Depth) e o lidar YDLidar (T-mini Plus) do WS500.

Uso (no Raspberry Pi / companion computer, via SSH):
  ros2 launch ws500_bringup sensors.launch.py

Comandos equivalentes documentados no Modulo 15 (DroneOS):
  ros2 launch orbbec_camera gemini_e.launch.py
  ros2 launch ydlidar_ros2_driver ydlidar_launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('orbbec_camera'), '/launch/gemini_e.launch.py'
        ]),
        # Explicito em vez de depender do default do launch incluido: o
        # color_tracker_node assume depth ja alinhado (mesmo pixel/resolucao)
        # ao frame de cor.
        launch_arguments={'depth_registration': 'true'}.items(),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ydlidar_ros2_driver'), '/launch/ydlidar_launch.py'
        ]),
    )

    return LaunchDescription([
        camera_launch,
        lidar_launch,
    ])
