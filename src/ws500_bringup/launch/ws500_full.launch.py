"""Bringup completo em um unico launch: mavros + sensores + percepcao + controle.

ATENCAO: isto sobe tudo de uma vez, incluindo o visual_servo_node que envia
setpoints continuos para /mavros/setpoint_raw/local. Isso so e seguro depois
de ja ter validado cada camada isoladamente pelo menos uma vez:
  1) ws500.launch.py sozinho -> conferir telemetria (/mavros/state) e imagem
  2) tracking.launch.py sozinho -> conferir deteccao (rqt_image_view em
     /ws500/ball/debug_image) e ganhos de controle, drone ainda desarmado
Os comandos de velocidade so tem efeito real quando o piloto de seguranca
troca para GUIDED pela chave do radio — mas o stream ja fica ativo assim
que este launch sobe.

Uso (no Raspberry Pi / companion computer, via SSH):
  ros2 launch ws500_bringup ws500_full.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ws500 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ws500_bringup'), '/launch/ws500.launch.py'
        ]),
    )

    tracking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ws500_bringup'), '/launch/tracking.launch.py'
        ]),
    )

    return LaunchDescription([
        ws500,
        tracking,
    ])
