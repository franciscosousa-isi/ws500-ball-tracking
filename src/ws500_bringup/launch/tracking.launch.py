"""Sobe os nos de percepcao (deteccao de cor) e controle (visual servoing).

Uso (no Raspberry Pi / companion computer, via SSH, com mavros.launch.py e
sensors.launch.py ja rodando em outros terminais):
  ros2 launch ws500_bringup tracking.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('ws500_bringup')
    color_tracker_params = os.path.join(bringup_share, 'config', 'color_tracker.yaml')
    visual_servo_params = os.path.join(bringup_share, 'config', 'visual_servo.yaml')

    color_tracker_node = Node(
        package='ws500_perception',
        executable='color_tracker_node',
        name='color_tracker_node',
        output='screen',
        parameters=[color_tracker_params],
    )

    visual_servo_node = Node(
        package='ws500_control',
        executable='visual_servo_node',
        name='visual_servo_node',
        output='screen',
        parameters=[visual_servo_params],
    )

    return LaunchDescription([
        color_tracker_node,
        visual_servo_node,
    ])
