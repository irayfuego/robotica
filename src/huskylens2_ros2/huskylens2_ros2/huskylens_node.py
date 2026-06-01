#!/usr/bin/env python3
"""
huskylens_node.py
Nodo ROS 2 para la HuskyLens 2 (DFRobot SEN0638 / Product 3118 Plus Kit).

Publica las detecciones de la cámara como mensajes ROS estándar.

Suscripciones:
    /huskylens/set_algorithm  (std_msgs/String)  — cambia algoritmo en caliente
                               Valores: 'face', 'object', 'line', 'color',
                                        'tag', 'gesture', 'pose', 'qrcode', etc.

Publicaciones:
    /huskylens/detections     (vision_msgs/Detection2DArray)  — resultados
    /huskylens/tracked_object (geometry_msgs/Point)           — centro objeto principal
    /huskylens/algorithm      (std_msgs/String)               — algoritmo activo

Parámetros:
    i2c_bus      (int)    1        — bus I2C (normalmente 1 en RPi4)
    i2c_address  (int)    0x32     — dirección I2C de la cámara
    algorithm    (string) 'object_recognition'
    poll_rate    (float)  10.0     — Hz de lectura de la cámara
    frame_id     (string) 'camera_link'
    image_width  (int)    320      — resolución de referencia para normalizar
    image_height (int)    240

Notas sobre el MCP Server (HuskyLens 2 Plus Kit con Wi-Fi):
    Este nodo usa la conexión I2C para publicar en ROS 2.
    El servidor MCP está integrado en el FIRMWARE del dispositivo y se accede
    via Wi-Fi directamente desde Claude Desktop u otros clientes LLM.

    Configuración del MCP (en el dispositivo):
        1. Settings → WiFi → conectar a tu red
        2. Settings → MCP Server → Enable
        3. Anotar la IP que muestra la pantalla
        4. En Claude Desktop, añadir en claude_desktop_config.json:
           {
             "mcpServers": {
               "huskylens": {
                 "url": "http://<IP_DE_LA_CAMARA>/mcp"
               }
             }
           }
    Herramientas MCP disponibles:
        - get_recognition_result  → foto + etiquetas en tiempo real
        - manage_applications     → listar/cambiar algoritmo
        - multimedia_control      → tomar foto
        - task_scheduler          → programar acciones por trigger
"""

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from huskylens2_ros2.dfrobot_huskylens_i2c import (
    HuskyLensI2C, ALGO_NAMES,
    ALGO_FACE_RECOGNITION, ALGO_OBJECT_TRACKING, ALGO_OBJECT_RECOGNITION,
    ALGO_LINE_TRACKING, ALGO_COLOR_RECOGNITION, ALGO_TAG_RECOGNITION,
    ALGO_GESTURE_RECOGNITION, ALGO_POSE_RECOGNITION, ALGO_HAND_RECOGNITION,
    ALGO_OCR_RECOGNITION, ALGO_QRCODE_RECOGNITION, ALGO_BARCODE_RECOGNITION,
)

# Mapa de nombre → constante de algoritmo
_ALGO_MAP = {
    'face':         ALGO_FACE_RECOGNITION,
    'face_recognition': ALGO_FACE_RECOGNITION,
    'object':       ALGO_OBJECT_RECOGNITION,
    'object_recognition': ALGO_OBJECT_RECOGNITION,
    'tracking':     ALGO_OBJECT_TRACKING,
    'object_tracking': ALGO_OBJECT_TRACKING,
    'line':         ALGO_LINE_TRACKING,
    'line_tracking': ALGO_LINE_TRACKING,
    'color':        ALGO_COLOR_RECOGNITION,
    'color_recognition': ALGO_COLOR_RECOGNITION,
    'tag':          ALGO_TAG_RECOGNITION,
    'tag_recognition': ALGO_TAG_RECOGNITION,
    'gesture':      ALGO_GESTURE_RECOGNITION,
    'gesture_recognition': ALGO_GESTURE_RECOGNITION,
    'pose':         ALGO_POSE_RECOGNITION,
    'pose_recognition': ALGO_POSE_RECOGNITION,
    'hand':         ALGO_HAND_RECOGNITION,
    'hand_recognition': ALGO_HAND_RECOGNITION,
    'ocr':          ALGO_OCR_RECOGNITION,
    'ocr_recognition': ALGO_OCR_RECOGNITION,
    'qr':           ALGO_QRCODE_RECOGNITION,
    'qrcode':       ALGO_QRCODE_RECOGNITION,
    'qrcode_recognition': ALGO_QRCODE_RECOGNITION,
    'barcode':      ALGO_BARCODE_RECOGNITION,
    'barcode_recognition': ALGO_BARCODE_RECOGNITION,
}


class HuskyLensNode(Node):

    def __init__(self):
        super().__init__('huskylens_node')

        # ── Parámetros ───────────────────────────────────────────────────────
        self.declare_parameter('i2c_bus',     1)
        self.declare_parameter('i2c_address', 0x32)
        self.declare_parameter('algorithm',   'object_recognition')
        self.declare_parameter('poll_rate',   10.0)
        self.declare_parameter('frame_id',    'camera_link')
        self.declare_parameter('image_width',  320)
        self.declare_parameter('image_height', 240)

        bus      = self.get_parameter('i2c_bus').value
        addr     = self.get_parameter('i2c_address').value
        algo_str = self.get_parameter('algorithm').value
        rate     = self.get_parameter('poll_rate').value
        self._frame_id = self.get_parameter('frame_id').value
        self._img_w    = self.get_parameter('image_width').value
        self._img_h    = self.get_parameter('image_height').value

        # ── Conexión I2C ─────────────────────────────────────────────────────
        self.get_logger().info(f'Conectando a HuskyLens 2 en I2C bus={bus} addr=0x{addr:02X}...')
        try:
            self._hl = HuskyLensI2C(bus, addr)
            if not self._hl.begin():
                self.get_logger().error('No se pudo comunicar con la HuskyLens 2. '
                                        'Comprueba el cableado I2C y la dirección.')
                raise RuntimeError('HuskyLens init failed')
            self.get_logger().info('✅ HuskyLens 2 conectada')
        except Exception as e:
            self.get_logger().error(f'❌ Error al conectar: {e}')
            raise

        # ── Algoritmo inicial ─────────────────────────────────────────────────
        self._current_algo_id   = _ALGO_MAP.get(algo_str, ALGO_OBJECT_RECOGNITION)
        self._current_algo_name = algo_str
        self._set_algorithm(self._current_algo_id)

        # ── Publicadores ─────────────────────────────────────────────────────
        self._det_pub   = self.create_publisher(
            Detection2DArray, 'huskylens/detections', 10)
        self._point_pub = self.create_publisher(
            Point, 'huskylens/tracked_object', 10)
        self._algo_pub  = self.create_publisher(
            String, 'huskylens/algorithm', 1)

        # ── Suscriptores ─────────────────────────────────────────────────────
        self.create_subscription(
            String, 'huskylens/set_algorithm', self._on_set_algorithm, 5)

        # ── Timer de polling ──────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._poll)

        self.get_logger().info(
            f'huskylens_node activo — algoritmo: {algo_str} @ {rate:.0f} Hz')
        self.get_logger().info(
            'MCP Server (via Wi-Fi): configurar en el dispositivo → Settings → MCP Server')

    def _set_algorithm(self, algo_id: int) -> bool:
        ok = self._hl.set_algorithm(algo_id)
        name = ALGO_NAMES.get(algo_id, str(algo_id))
        if ok:
            self._current_algo_id   = algo_id
            self._current_algo_name = name
            self.get_logger().info(f'Algoritmo activo: {name}')
        else:
            self.get_logger().warn(f'No se pudo cambiar al algoritmo: {name}')
        return ok

    def _on_set_algorithm(self, msg: String):
        """Cambia el algoritmo desde un topic ROS."""
        algo_str = msg.data.strip().lower()
        algo_id  = _ALGO_MAP.get(algo_str)
        if algo_id is None:
            self.get_logger().warn(
                f'Algoritmo desconocido: "{algo_str}". '
                f'Disponibles: {list(_ALGO_MAP.keys())}')
            return
        self._set_algorithm(algo_id)

    def _poll(self):
        """Solicita resultados y publica."""
        try:
            results = self._hl.get_results()
        except Exception as e:
            self.get_logger().warn(f'Error al leer HuskyLens: {e}', throttle_duration_sec=5.0)
            return

        now = self.get_clock().now().to_msg()

        # ── Detection2DArray ─────────────────────────────────────────────────
        det_array = Detection2DArray()
        det_array.header.stamp    = now
        det_array.header.frame_id = self._frame_id

        for r in results:
            if not r.is_block:
                continue
            d = Detection2D()
            d.header = det_array.header
            d.bbox.center.position.x = float(r.x_center)
            d.bbox.center.position.y = float(r.y_center)
            d.bbox.size_x = float(r.width)
            d.bbox.size_y = float(r.height)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(r.id)
            hyp.hypothesis.score    = 1.0
            d.results.append(hyp)
            det_array.detections.append(d)

        self._det_pub.publish(det_array)

        # ── Punto del objeto principal (id más bajo o primero) ────────────────
        if results:
            primary = min((r for r in results if r.is_block),
                          key=lambda r: r.id, default=None)
            if primary is not None:
                pt = Point()
                # Normalizar a [-1, 1] centrado en imagen
                pt.x = (primary.x_center - self._img_w / 2.0) / (self._img_w / 2.0)
                pt.y = (primary.y_center - self._img_h / 2.0) / (self._img_h / 2.0)
                pt.z = 0.0
                self._point_pub.publish(pt)

        # ── Publicar algoritmo activo ─────────────────────────────────────────
        algo_msg = String()
        algo_msg.data = self._current_algo_name
        self._algo_pub.publish(algo_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HuskyLensNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
