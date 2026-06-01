#!/usr/bin/env python3
"""
huskylens_bridge_node.py — Bridge HuskyLens 2 → robot_eyes (ROS 2 Jazzy)
Migrado de ROS 1 (rospy) a ROS 2 (rclpy).

Modos:
  0 - FACE_RECOGNITION      → sigue la cara
  1 - OBJECT_TRACKING       → sigue objeto aprendido
  2 - OBJECT_RECOGNITION    → emoción por clase
  3 - LINE_TRACKING         → ojos siguen la línea
  4 - COLOR_RECOGNITION     → iris cambia de color
  5 - TAG_RECOGNITION       → comportamiento por ID tag
  6 - EMOTION_RECOGNITION   → ojos reflejan la emoción detectada (default)

Cableado UART (recomendado RPi 4B):
  HuskyLens TX → RPi GPIO15 (RXD)  /dev/serial0
  HuskyLens RX → RPi GPIO14 (TXD)
  GND → GND,  VCC → 5V
  raspi-config → Interface → Serial → disable shell, enable hardware serial
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point

HUSKY_W = 320.0
HUSKY_H = 240.0

# ── Mapa emoción ID → configuración de ojos ───────────────────────────────────
HUSKY_EMOTION_MAP = {
    1: {'behavior': 'angry',      'iris': (210, 50,  40),  'extra': None,         'note': 'Angry'},
    2: {'behavior': 'suspicious', 'iris': (120, 100, 40),  'extra': None,         'note': 'Disgust'},
    3: {'behavior': 'surprised',  'iris': (180, 200, 255), 'extra': 'dilate',     'note': 'Fear'},
    4: {'behavior': 'happy',      'iris': (60,  200, 100), 'extra': None,         'note': 'Happiness'},
    5: {'behavior': 'look_center','iris': (60,  120, 200), 'extra': None,         'note': 'Neutral'},
    6: {'behavior': 'sad',        'iris': (50,  70,  180), 'extra': 'slow_blink', 'note': 'Sadness'},
    7: {'behavior': 'surprised',  'iris': (100, 180, 255), 'extra': None,         'note': 'Surprise'},
}

OBJECT_ID_MAP = {
    1: {'behavior': 'happy',     'iris': (80,  180, 80)},
    2: {'behavior': 'surprised', 'iris': (100, 160, 220)},
    3: {'behavior': 'angry',     'iris': (200, 50,  40)},
    4: {'behavior': 'love',      'iris': (220, 80,  130)},
    5: {'behavior': 'sad',       'iris': (50,  70,  160)},
}

COLOR_ID_MAP = {
    1: (200, 60,  60),
    2: (60,  200, 80),
    3: (60,  100, 220),
    4: (220, 180, 40),
    5: (180, 60,  200),
}


class HuskyLensBridgeNode(Node):

    def __init__(self):
        super().__init__('huskylens_bridge_node')

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter('interface',    'uart')
        self.declare_parameter('port',         '/dev/serial0')
        self.declare_parameter('baudrate',     9600)
        self.declare_parameter('address',      0x32)
        self.declare_parameter('mode',         6)
        self.declare_parameter('rate',         20.0)
        self.declare_parameter('lost_timeout', 1.5)
        self.declare_parameter('confidence',   60)
        self.declare_parameter('emotion_hold', 0.8)

        self.interface    = self.get_parameter('interface').value
        self.port         = self.get_parameter('port').value
        self.baudrate     = self.get_parameter('baudrate').value
        self.i2c_addr     = self.get_parameter('address').value
        self.hl_mode      = self.get_parameter('mode').value
        self.rate_hz      = self.get_parameter('rate').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.confidence   = self.get_parameter('confidence').value
        self.emotion_hold = self.get_parameter('emotion_hold').value

        # ── Publicadores ──────────────────────────────────────────────────────
        self.pub_face  = self.create_publisher(Point,     '/robot_eyes/face_position', 1)
        self.pub_beh   = self.create_publisher(String,    '/robot_eyes/behavior',      1)
        self.pub_color = self.create_publisher(ColorRGBA, '/robot_eyes/iris_color',    1)

        # ── Estado interno ────────────────────────────────────────────────────
        self._last_seen         = 0.0
        self._target_visible    = False
        self._current_emotion   = None
        self._emotion_since     = 0.0
        self._last_emotion_sent = None
        self._extra_timer       = None

        # ── Iniciar HuskyLens ─────────────────────────────────────────────────
        self._hl = None
        self._init_huskylens()

        # ── Timer de poll ─────────────────────────────────────────────────────
        period = 1.0 / self.rate_hz
        self._handler_map = {
            0: self._handle_face_recognition,
            1: self._handle_object_tracking,
            2: self._handle_object_recognition,
            3: self._handle_line_tracking,
            4: self._handle_color_recognition,
            5: self._handle_tag_recognition,
            6: self._handle_emotion_recognition,
        }
        self.create_timer(period, self._poll)
        self.get_logger().info('HuskyLens bridge en marcha.')

    # ── Init HuskyLens ────────────────────────────────────────────────────────
    def _init_huskylens(self):
        try:
            from huskylens import HuskyLens
        except ImportError:
            self.get_logger().fatal('Instala la librería: pip install huskylens')
            raise SystemExit(1)

        self.get_logger().info(f'Conectando HuskyLens 2 via {self.interface.upper()}...')

        if self.interface == 'uart':
            import serial
            ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self._hl = HuskyLens()
            self._hl.connect(ser)
        elif self.interface == 'i2c':
            import smbus2
            self._hl = HuskyLens()
            self._hl.connect(smbus2.SMBus(1), address=self.i2c_addr)
        else:
            self.get_logger().fatal(f'Interface desconocida: {self.interface}')
            raise SystemExit(1)

        algo_map = {
            0: 'ALGORITHM_FACE_RECOGNITION',
            1: 'ALGORITHM_OBJECT_TRACKING',
            2: 'ALGORITHM_OBJECT_RECOGNITION',
            3: 'ALGORITHM_LINE_TRACKING',
            4: 'ALGORITHM_COLOR_RECOGNITION',
            5: 'ALGORITHM_TAG_RECOGNITION',
            6: 'ALGORITHM_FACE_EMOTION_RECOGNITION',
        }
        algo = algo_map.get(self.hl_mode, 'ALGORITHM_FACE_EMOTION_RECOGNITION')
        self._hl.set_algorithm(algo)
        self.get_logger().info(f'HuskyLens 2 lista. Modo: {algo}')
        time.sleep(0.3)
        self._pub_beh_str('wake_up')

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _normalize(self, x, y):
        return (x - HUSKY_W / 2) / (HUSKY_W / 2), (y - HUSKY_H / 2) / (HUSKY_H / 2)

    def _pub_beh_str(self, name: str):
        msg = String(); msg.data = name
        self.pub_beh.publish(msg)

    def _set_iris(self, r, g, b):
        msg = ColorRGBA()
        msg.r = r / 255.0; msg.g = g / 255.0
        msg.b = b / 255.0; msg.a = 1.0
        self.pub_color.publish(msg)

    def _pub_point(self, x, y, z=1.0):
        msg = Point(); msg.x = float(x); msg.y = float(y); msg.z = float(z)
        self.pub_face.publish(msg)

    # ── Modo 6: Emotion Recognition ───────────────────────────────────────────
    def _handle_emotion_recognition(self, blocks):
        if not blocks:
            return False
        target = max(blocks, key=lambda b: b.width * b.height)
        nx, ny = self._normalize(target.x, target.y)
        self._pub_point(nx, ny)

        emotion_id = target.ID
        mapping    = HUSKY_EMOTION_MAP.get(emotion_id)
        if not mapping:
            return True

        now = time.time()
        if emotion_id != self._current_emotion:
            self._current_emotion = emotion_id
            self._emotion_since   = now
            self.get_logger().info(
                f'Emoción detectada: {mapping["note"]} (ID={emotion_id})',
                throttle_duration_sec=1.0)
            return True

        if (now - self._emotion_since) >= self.emotion_hold \
                and emotion_id != self._last_emotion_sent:
            self.get_logger().info(f'Reflejando emoción: {mapping["note"]}')
            self._set_iris(*mapping['iris'])
            self._pub_beh_str(mapping['behavior'])
            if mapping['extra']:
                # Lanzar el comportamiento extra con un pequeño delay via timer one-shot
                extra_name = mapping['extra']
                if self._extra_timer:
                    self._extra_timer.cancel()
                self._extra_timer = self.create_timer(
                    0.6,
                    lambda n=extra_name: (self._pub_beh_str(n),
                                          self._extra_timer.cancel())
                )
            self._last_emotion_sent = emotion_id
            if not self._target_visible:
                self._target_visible = True
        return True

    # ── Resto de modos ────────────────────────────────────────────────────────
    def _handle_face_recognition(self, blocks):
        if not blocks:
            return False
        target = max(blocks, key=lambda b: b.width * b.height)
        nx, ny = self._normalize(target.x, target.y)
        self._pub_point(nx, ny)
        if not self._target_visible:
            self._pub_beh_str('notice')
            self._target_visible = True
        return True

    def _handle_object_tracking(self, blocks):
        if not blocks:
            return False
        nx, ny = self._normalize(blocks[0].x, blocks[0].y)
        self._pub_point(nx, ny)
        return True

    def _handle_object_recognition(self, blocks):
        if not blocks:
            return False
        target = blocks[0]
        nx, ny = self._normalize(target.x, target.y)
        self._pub_point(nx, ny)
        m = OBJECT_ID_MAP.get(target.ID)
        if m and not self._target_visible:
            self._pub_beh_str(m['behavior'])
            self._set_iris(*m['iris'])
        return True

    def _handle_color_recognition(self, blocks):
        if not blocks:
            return False
        c = COLOR_ID_MAP.get(blocks[0].ID)
        if c:
            self._set_iris(*c)
        nx, ny = self._normalize(blocks[0].x, blocks[0].y)
        self._pub_point(nx, ny)
        return True

    def _handle_tag_recognition(self, blocks):
        if not blocks:
            return False
        m = OBJECT_ID_MAP.get(blocks[0].ID)
        if m:
            self._pub_beh_str(m['behavior'])
        return True

    def _handle_line_tracking(self, arrows):
        if not arrows:
            return False
        a = arrows[0]
        dx = (a.x2 - a.x1) / HUSKY_W
        dy = (a.y2 - a.y1) / HUSKY_H
        self._pub_point(dx, dy)
        return True

    # ── Poll timer ────────────────────────────────────────────────────────────
    def _poll(self):
        if self._hl is None:
            return
        handler  = self._handler_map.get(self.hl_mode, self._handle_emotion_recognition)
        detected = False
        try:
            data = self._hl.arrows() if self.hl_mode == 3 else self._hl.blocks()
            if data:
                detected = handler(data)
        except Exception as e:
            self.get_logger().warn(f'HuskyLens read error: {e}',
                                   throttle_duration_sec=5.0)

        if detected:
            self._last_seen = time.time()
        elif self._target_visible and \
                (time.time() - self._last_seen) > self.lost_timeout:
            self.get_logger().info('Objetivo perdido → centro + neutral')
            self._pub_beh_str('look_center')
            self._pub_point(0.0, 0.0, 0.0)
            self._set_iris(60, 120, 200)
            self._target_visible    = False
            self._current_emotion   = None
            self._last_emotion_sent = None


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = HuskyLensBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
