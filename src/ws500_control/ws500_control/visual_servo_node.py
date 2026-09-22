import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)


def clamp(value: float, low: float, high: float) -> float:
    if not math.isfinite(value):
        return 0.0
    return max(low, min(high, value))


class VisualServoNode(Node):
    """Mantem distancia e altitude em relacao a bola detectada, via MAVROS/GUIDED.

    So publica comandos de velocidade uteis quando o piloto de seguranca ja
    colocou o ArduPilot em modo GUIDED (troca feita pela chave fisica no radio,
    fora do escopo deste no). Sem deteccao recente da bola, o no publica
    velocidade zero (hover) em vez de tentar adivinhar a posicao do alvo.
    """

    def __init__(self):
        super().__init__('visual_servo_node')

        self.declare_parameter('target_distance_m', 1.0)   # centro da faixa 0.5-1.5m
        self.declare_parameter('min_altitude_m', 1.0)
        self.declare_parameter('max_altitude_m', 3.0)
        self.declare_parameter('ball_lost_timeout_s', 1.0)
        self.declare_parameter('control_rate_hz', 20.0)

        self.declare_parameter('kp_distance', 0.6)
        self.declare_parameter('kp_yaw', 1.2)
        self.declare_parameter('kp_altitude', 0.8)

        self.declare_parameter('max_vx_mps', 0.6)
        self.declare_parameter('max_vz_mps', 0.4)
        self.declare_parameter('max_yaw_rate_rps', 0.8)

        self.declare_parameter('ball_position_topic', '/ws500/ball/position')

        self._last_ball: PointStamped | None = None
        self._current_mode: str = ''
        self._armed: bool = False
        self._current_alt_m: float | None = None

        # QoS de /mavros/state: mavros::StateQoS() = QoS(10).transient_local()
        # (reliable). /mavros/local_position/pose: qos_profile_sensor_data
        # (best_effort). Usar o QoS default (reliable) do rclpy nesses dois
        # topicos causaria incompatibilidade silenciosa — sem excecao, sem
        # log, so nenhuma mensagem chegando. No caso da pose isso desativaria
        # o clamp de altitude de seguranca sem nenhum aviso.
        qos_state = QoSProfile(depth=10)
        qos_state.reliability = QoSReliabilityPolicy.RELIABLE
        qos_state.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            PointStamped, self.get_parameter('ball_position_topic').value,
            self._on_ball, 10)
        self.create_subscription(State, '/mavros/state', self._on_state, qos_state)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._on_local_pose,
            qos_profile_sensor_data)

        self._setpoint_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10)

        rate_hz = self.get_parameter('control_rate_hz').value
        self._timer = self.create_timer(1.0 / rate_hz, self._on_control_tick)

        self.get_logger().info('VisualServoNode pronto.')

    def _on_ball(self, msg: PointStamped) -> None:
        self._last_ball = msg

    def _on_state(self, msg: State) -> None:
        self._current_mode = msg.mode
        self._armed = msg.armed

    def _on_local_pose(self, msg: PoseStamped) -> None:
        self._current_alt_m = msg.pose.position.z

    def _ball_is_fresh(self) -> bool:
        if self._last_ball is None:
            return False
        # Idade medida pelo timestamp da propria mensagem (header.stamp),
        # nao pelo instante em que este no a recebeu — senao um atraso no
        # pipeline de percepcao (sincronizacao, processamento) fica invisivel
        # e uma deteccao velha passa por "fresca".
        msg_time = Time.from_msg(self._last_ball.header.stamp)
        age = (self.get_clock().now() - msg_time).nanoseconds / 1e9
        return 0.0 <= age <= self.get_parameter('ball_lost_timeout_s').value

    def _clamp_vertical_for_altitude(self, vz_up: float) -> float:
        """vz_up positivo = subir. Impede sair de [min_altitude, max_altitude].

        Sem leitura de altitude ainda, nao ha como saber se estamos perto dos
        limites — trava o eixo vertical (0.0) em vez de confiar cegamente no
        offset vertical da bola.
        """
        if self._current_alt_m is None:
            return 0.0

        min_alt = self.get_parameter('min_altitude_m').value
        max_alt = self.get_parameter('max_altitude_m').value

        if self._current_alt_m <= min_alt and vz_up < 0.0:
            return 0.0
        if self._current_alt_m >= max_alt and vz_up > 0.0:
            return 0.0
        return vz_up

    def _compute_setpoint(self) -> PositionTarget:
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'base_link'
        sp.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED
        sp.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )

        if not self._ball_is_fresh():
            sp.velocity.x = 0.0
            sp.velocity.y = 0.0
            sp.velocity.z = 0.0
            sp.yaw_rate = 0.0
            return sp

        ball = self._last_ball.point  # x: lateral(m), y: vertical(m), z: distancia(m)

        target_distance = self.get_parameter('target_distance_m').value
        distance_error = ball.z - target_distance
        vx = clamp(
            self.get_parameter('kp_distance').value * distance_error,
            -self.get_parameter('max_vx_mps').value,
            self.get_parameter('max_vx_mps').value,
        )

        # yaw_rate: mavros SEMPRE converte este campo de ENU (positivo =
        # anti-horario/esquerda) para NED antes de mandar pro FC, mesmo em
        # frame BODY_*. bola.x>0 (alvo a direita) precisa virar a direita
        # (NED positivo) => temos que mandar ENU NEGATIVO. Por isso o sinal
        # trocado aqui, nao e engano.
        yaw_error = math.atan2(ball.x, ball.z)
        yaw_rate = clamp(
            -self.get_parameter('kp_yaw').value * yaw_error,
            -self.get_parameter('max_yaw_rate_rps').value,
            self.get_parameter('max_yaw_rate_rps').value,
        )

        # velocity.z em FRAME_BODY_OFFSET_NED: mavros espera baselink/FLU
        # (Z para CIMA positivo) e converte pra FRD internamente. bola.y>0
        # (alvo abaixo do centro da imagem) precisa descer => vz_up negativo.
        vz_up = clamp(
            -self.get_parameter('kp_altitude').value * ball.y,
            -self.get_parameter('max_vz_mps').value,
            self.get_parameter('max_vz_mps').value,
        )
        vz_up = self._clamp_vertical_for_altitude(vz_up)

        sp.velocity.x = vx
        sp.velocity.y = 0.0
        sp.velocity.z = vz_up
        sp.yaw_rate = yaw_rate
        return sp

    def _on_control_tick(self) -> None:
        setpoint = self._compute_setpoint()

        if self._current_mode != 'GUIDED' or not self._armed:
            setpoint.velocity.x = 0.0
            setpoint.velocity.y = 0.0
            setpoint.velocity.z = 0.0
            setpoint.yaw_rate = 0.0

        self._setpoint_pub.publish(setpoint)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
