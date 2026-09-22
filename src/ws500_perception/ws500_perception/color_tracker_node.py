import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class ColorTrackerNode(Node):
    """Detecta a bola vermelha (RGB) e publica sua posicao 3D no frame da camera.

    Saida (/ws500/ball/position, geometry_msgs/PointStamped, so quando detectada):
      x = deslocamento lateral em metros (direita positivo)
      y = deslocamento vertical em metros (baixo positivo, convencao da imagem)
      z = distancia estimada em metros (profundidade)
    """

    def __init__(self):
        super().__init__('color_tracker_node')

        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('output_topic', '/ws500/ball/position')
        self.declare_parameter('debug_image_topic', '/ws500/ball/debug_image')

        # HSV do vermelho: dois intervalos porque o matiz vermelho cruza 0/180.
        self.declare_parameter('hsv_lower1', [0, 120, 70])
        self.declare_parameter('hsv_upper1', [10, 255, 255])
        self.declare_parameter('hsv_lower2', [170, 120, 70])
        self.declare_parameter('hsv_upper2', [180, 255, 255])

        self.declare_parameter('min_contour_area_px', 150.0)
        self.declare_parameter('ball_diameter_m', 0.15)
        self.declare_parameter('depth_roi_px', 5)

        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None

        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        # QoS best_effort: compativel com publishers best_effort OU reliable.
        # Um subscriber reliable (o default do rclpy) nao recebe nada de um
        # publisher best_effort — e a maioria dos drivers de camera publica
        # imagem em best_effort. Isso falha silenciosamente (sem erro).
        self._info_sub = self.create_subscription(
            CameraInfo, info_topic, self._on_camera_info, qos_profile_sensor_data)

        self._color_sub = Subscriber(self, Image, color_topic, qos_profile=qos_profile_sensor_data)
        self._depth_sub = Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        self._sync = ApproximateTimeSynchronizer(
            [self._color_sub, self._depth_sub], queue_size=5, slop=0.05)
        self._sync.registerCallback(self._on_frames)

        self._pos_pub = self.create_publisher(
            PointStamped, self.get_parameter('output_topic').value, 10)
        self._debug_pub = self.create_publisher(
            Image, self.get_parameter('debug_image_topic').value, 1)

        self.get_logger().info(
            f'ColorTrackerNode pronto. color={color_topic} depth={depth_topic} info={info_topic}')

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _red_mask(self, bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lo1 = np.array(self.get_parameter('hsv_lower1').value, dtype=np.uint8)
        hi1 = np.array(self.get_parameter('hsv_upper1').value, dtype=np.uint8)
        lo2 = np.array(self.get_parameter('hsv_lower2').value, dtype=np.uint8)
        hi2 = np.array(self.get_parameter('hsv_upper2').value, dtype=np.uint8)
        mask = cv2.inRange(hsv, lo1, hi1) | cv2.inRange(hsv, lo2, hi2)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def _depth_at(self, depth: np.ndarray, u: int, v: int) -> float:
        roi = self.get_parameter('depth_roi_px').value
        h, w = depth.shape[:2]
        u0, u1 = max(0, u - roi), min(w, u + roi + 1)
        v0, v1 = max(0, v - roi), min(h, v + roi + 1)
        patch = depth[v0:v1, u0:u1].astype(np.float32)

        if depth.dtype == np.uint16:
            patch = patch / 1000.0  # mm -> m

        valid = patch[(patch > 0.05) & np.isfinite(patch)]
        if valid.size == 0:
            return float('nan')
        return float(np.median(valid))

    def _on_frames(self, color_msg: Image, depth_msg: Image) -> None:
        if self._camera_info is None:
            return

        bgr = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        mask = self._red_mask(bgr)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        debug = bgr.copy()

        if contours:
            best = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(best)
            if area >= self.get_parameter('min_contour_area_px').value:
                (u, v), radius_px = cv2.minEnclosingCircle(best)
                u, v = int(u), int(v)

                fx = self._camera_info.k[0]
                fy = self._camera_info.k[4]
                cx = self._camera_info.k[2]
                cy = self._camera_info.k[5]

                z = self._depth_at(depth, u, v)
                used_fallback = False
                if not np.isfinite(z) or z <= 0.0:
                    diameter_m = self.get_parameter('ball_diameter_m').value
                    z = (diameter_m * fx) / (2.0 * radius_px) if radius_px > 1e-3 else float('nan')
                    used_fallback = True

                x = y = float('nan')
                if np.isfinite(z) and z > 0.0 and fx > 1e-3 and fy > 1e-3:
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy

                if np.isfinite(x) and np.isfinite(y) and np.isfinite(z) and z > 0.0:
                    msg = PointStamped()
                    msg.header = color_msg.header
                    msg.point.x = x
                    msg.point.y = y
                    msg.point.z = z
                    self._pos_pub.publish(msg)

                    color = (0, 165, 255) if used_fallback else (0, 255, 0)
                    cv2.circle(debug, (u, v), int(radius_px), color, 2)
                    cv2.drawMarker(debug, (u, v), (255, 0, 0), cv2.MARKER_CROSS, 12, 2)
                    cv2.putText(
                        debug, f'z={z:.2f}m{" (fallback)" if used_fallback else ""}',
                        (u + 10, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        self._debug_pub.publish(self._bridge.cv2_to_imgmsg(debug, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = ColorTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
