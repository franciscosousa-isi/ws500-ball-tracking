"""Sobe o MAVROS conectado ao Cube Orange (ArduPilot) do WS500.

Uso (no Raspberry Pi / companion computer, via SSH):
  ros2 launch ws500_bringup mavros.launch.py

Valores default replicam o procedimento documentado no Modulo 15 (DroneOS):
fcu_url = serial:///dev/ttyACM0:57600 (Cube Orange via USB)
gcs_url = udp://@<IP_DA_ESTACAO_SOLO>:14550 (Mission Planner, opcional)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='serial:///dev/ttyACM0:57600',
        description='Conexao com o Cube Orange (ArduPilot)',
    )
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='Encaminhamento opcional para Mission Planner (udp://@<IP>:14550)',
    )

    mavros_apm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('mavros'), '/launch/apm.launch'
        ]),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
        }.items(),
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        mavros_apm_launch,
    ])
