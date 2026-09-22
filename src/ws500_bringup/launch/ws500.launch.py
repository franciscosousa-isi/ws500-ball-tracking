"""Bringup completo do WS500 para o desafio de rastreamento da bola vermelha.

Sobe: MAVROS (Cube Orange/ArduPilot) + sensores (camera Orbbec + lidar YDLidar).

Os nos de percepcao/controle (tracking.launch.py) sao propositalmente
DEIXADOS DE FORA daqui: a sequencia recomendada e validar cada camada
isoladamente antes de armar o drone —
  1) este launch (mavros + sensores) e confirmar telemetria/imagem OK
  2) ros2 launch ws500_bringup tracking.launch.py e validar a deteccao
     (rqt_image_view em /ws500/ball/debug_image) e os ganhos de controle
     ANTES de armar e trocar para GUIDED pela chave do radio.

Uso (no Raspberry Pi / companion computer, via SSH):
  ros2 launch ws500_bringup ws500.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mavros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ws500_bringup'), '/launch/mavros.launch.py'
        ]),
    )

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ws500_bringup'), '/launch/sensors.launch.py'
        ]),
    )

    return LaunchDescription([
        mavros,
        sensors,
    ])
