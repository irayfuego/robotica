#!/usr/bin/env python3
"""
huskylens_bridge.py
-------------------
Nodo ROS que lee la HuskyLens 2 y publica en los topics de robot_eyes.

Modos HuskyLens 2 soportados:
  0 - FACE_RECOGNITION      → sigue la cara
  1 - OBJECT_TRACKING       → sigue objeto aprendido
  2 - OBJECT_RECOGNITION    → emoción por clase
  3 - LINE_TRACKING         → ojos siguen la línea
  4 - COLOR_RECOGNITION     → iris cambia de color
  5 - TAG_RECOGNITION       → comportamiento por ID tag
  6 - EMOTION_RECOGNITION   → *** ojos reflejan la emoción detectada ***

Conexión UART (recomendada RPi 4B):
  HuskyLens TX → RPi GPIO15 (RXD)   /dev/serial0
  HuskyLens RX → RPi GPIO14 (TXD)
  GND → GND,  VCC → 5V
  sudo raspi-config → Interface → Serial → disable shell, enable hardware serial

Conexión I2C:
  SDA → GPIO2,  SCL → GPIO3

Instalar:
  pip install huskylens pyserial smbus2

Uso:
  # Seguimiento de cara
  roslaunch robot_eyes robot_eyes_huskylens.launch mode:=0

  # Modo emoción (¡nuevo!)
  roslaunch robot_eyes robot_eyes_huskylens.launch mode:=6
"""

import time
import rospy
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point

HUSKY_W = 320.0
HUSKY_H = 240.0

# ---------------------------------------------------------------------------
# HuskyLens 2 Emotion Recognition
# ID devuelto por la HuskyLens → configuración de robot_eyes
# ---------------------------------------------------------------------------
HUSKY_EMOTION_MAP = {
    # ID  : (behavior_eyes,  iris_R, iris_G, iris_B, mirrored_blink, note)
    1: {
        "behavior":   "angry",
        "iris":       (210, 50,  40),
        "extra":      None,
        "note":       "Angry",
    },
    2: {
        "behavior":   "suspicious",   # disgusto → ojo entornado
        "iris":       (120, 100, 40),
        "extra":      None,
        "note":       "Disgust",
    },
    3: {
        "behavior":   "surprised",    # miedo → ojos muy abiertos
        "iris":       (180, 200, 255),
        "extra":      "dilate",       # además: pupilas dilatadas
        "note":       "Fear",
    },
    4: {
        "behavior":   "happy",
        "iris":       (60,  200, 100),
        "extra":      None,
        "note":       "Happiness",
    },
    5: {
        "behavior":   "look_center",  # neutral → reposo
        "iris":       (60,  120, 200),
        "extra":      None,
        "note":       "Neutral",
    },
    6: {
        "behavior":   "sad",
        "iris":       (50,  70,  180),
        "extra":      "slow_blink",   # además: parpadeo lento
        "note":       "Sadness",
    },
    7: {
        "behavior":   "surprised",
        "iris":       (100, 180, 255),
        "extra":      None,
        "note":       "Surprise",
    },
}

# Umbral mínimo de confianza para reaccionar (0-100)
CONFIDENCE_THRESHOLD = 60

# Segundos que debe mantenerse una emoción antes de reflejarla
# (evita parpadeos si la detección oscila)
EMOTION_HOLD_SECS = 0.8

# ---------------------------------------------------------------------------
# Mapa objeto ID → comportamiento (para modo OBJECT_RECOGNITION)
# ---------------------------------------------------------------------------
OBJECT_ID_MAP = {
    1: {"behavior": "happy",     "iris": (80,  180, 80)},
    2: {"behavior": "surprised", "iris": (100, 160, 220)},
    3: {"behavior": "angry",     "iris": (200, 50,  40)},
    4: {"behavior": "love",      "iris": (220, 80,  130)},
    5: {"behavior": "sad",       "iris": (50,  70,  160)},
}

COLOR_ID_MAP = {
    1: (200, 60,  60),
    2: (60,  200, 80),
    3: (60,  100, 220),
    4: (220, 180, 40),
    5: (180, 60,  200),
}


class HuskyLensBridge:

    def __init__(self):
        rospy.init_node("huskylens_bridge", anonymous=False)

        # Parámetros
        self.interface    = rospy.get_param("~interface",    "uart")
        self.port         = rospy.get_param("~port",         "/dev/serial0")
        self.baudrate     = rospy.get_param("~baudrate",     9600)
        self.i2c_addr     = rospy.get_param("~address",      0x32)
        self.hl_mode      = rospy.get_param("~mode",         6)    # default: emotion
        self.rate_hz      = rospy.get_param("~rate",         20)
        self.lost_timeout = rospy.get_param("~lost_timeout", 1.5)
        self.confidence   = rospy.get_param("~confidence",   CONFIDENCE_THRESHOLD)
        self.emotion_hold = rospy.get_param("~emotion_hold", EMOTION_HOLD_SECS)

        # Publishers
        self.pub_face  = rospy.Publisher("/robot_eyes/face_position", Point,     queue_size=1)
        self.pub_beh   = rospy.Publisher("/robot_eyes/behavior",      String,    queue_size=1)
        self.pub_color = rospy.Publisher("/robot_eyes/iris_color",    ColorRGBA, queue_size=1)

        # Estado
        self._last_seen         = 0.0
        self._target_visible    = False
        self._current_emotion   = None
        self._emotion_since     = 0.0   # cuándo empezó la emoción actual
        self._last_emotion_sent = None  # última emoción enviada a robot_eyes

        self._hl = None
        self._init_huskylens()

    # -----------------------------------------------------------------------
    # Init
    # -----------------------------------------------------------------------
    def _init_huskylens(self):
        try:
            from huskylens import HuskyLens
        except ImportError:
            rospy.logfatal("pip install huskylens")
            raise SystemExit(1)

        rospy.loginfo(f"Conectando HuskyLens 2 via {self.interface.upper()}...")

        if self.interface == "uart":
            import serial
            ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self._hl = HuskyLens()
            self._hl.connect(ser)
        elif self.interface == "i2c":
            import smbus2
            self._hl = HuskyLens()
            self._hl.connect(smbus2.SMBus(1), address=self.i2c_addr)
        else:
            rospy.logfatal(f"Interface desconocida: {self.interface}")
            raise SystemExit(1)

        algo_map = {
            0: "ALGORITHM_FACE_RECOGNITION",
            1: "ALGORITHM_OBJECT_TRACKING",
            2: "ALGORITHM_OBJECT_RECOGNITION",
            3: "ALGORITHM_LINE_TRACKING",
            4: "ALGORITHM_COLOR_RECOGNITION",
            5: "ALGORITHM_TAG_RECOGNITION",
            6: "ALGORITHM_FACE_EMOTION_RECOGNITION",
        }
        algo = algo_map.get(self.hl_mode, "ALGORITHM_FACE_EMOTION_RECOGNITION")
        self._hl.set_algorithm(algo)
        rospy.loginfo(f"HuskyLens 2 lista. Modo: {algo}")
        time.sleep(0.3)
        self.pub_beh.publish(String(data="wake_up"))

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------
    def _normalize(self, x, y):
        return (x - HUSKY_W / 2) / (HUSKY_W / 2), (y - HUSKY_H / 2) / (HUSKY_H / 2)

    def _set_iris(self, r, g, b):
        msg = ColorRGBA(r=r/255.0, g=g/255.0, b=b/255.0, a=1.0)
        self.pub_color.publish(msg)

    def _publish_behavior(self, name):
        self.pub_beh.publish(String(data=name))

    # -----------------------------------------------------------------------
    # Modo 6: Emotion Recognition  ← núcleo nuevo
    # -----------------------------------------------------------------------
    def _handle_emotion_recognition(self, blocks):
        """
        La HuskyLens 2 devuelve bloques con:
          block.ID         → ID de emoción (1-7)
          block.x, block.y → posición de la cara en el frame
          block.width, block.height → tamaño del bounding box
          (el score de confianza se parsea del label si está disponible)
        """
        if not blocks:
            return False

        # Cara más grande (más cercana)
        target = max(blocks, key=lambda b: b.width * b.height)

        # Posición → los ojos siguen la cara
        nx, ny = self._normalize(target.x, target.y)
        self.pub_face.publish(Point(x=nx, y=ny, z=1.0))

        emotion_id = target.ID
        mapping = HUSKY_EMOTION_MAP.get(emotion_id)
        if not mapping:
            return True

        now = time.time()

        # Lógica de estabilidad: solo reaccionar si la misma emoción
        # se mantiene EMOTION_HOLD_SECS seguidos (evita parpadeo)
        if emotion_id != self._current_emotion:
            self._current_emotion = emotion_id
            self._emotion_since   = now
            rospy.loginfo_throttle(1.0, f"Emoción detectada: {mapping['note']} (ID={emotion_id})")
            return True

        time_held = now - self._emotion_since
        if time_held >= self.emotion_hold and emotion_id != self._last_emotion_sent:
            # ¡Reaccionar!
            rospy.loginfo(f"Reflejando emoción: {mapping['note']}")

            # 1. Cambiar color de iris
            self._set_iris(*mapping["iris"])

            # 2. Comportamiento principal
            self._publish_behavior(mapping["behavior"])

            # 3. Comportamiento extra opcional (con pequeño delay)
            if mapping["extra"]:
                rospy.Timer(
                    rospy.Duration(0.6),
                    lambda e, b=mapping["extra"]: self._publish_behavior(b),
                    oneshot=True
                )

            self._last_emotion_sent = emotion_id

            # Primera vez que aparece un objetivo
            if not self._target_visible:
                self._target_visible = True

        return True

    # -----------------------------------------------------------------------
    # Resto de modos
    # -----------------------------------------------------------------------
    def _handle_face_recognition(self, blocks):
        if not blocks:
            return False
        target = max(blocks, key=lambda b: b.width * b.height)
        nx, ny = self._normalize(target.x, target.y)
        self.pub_face.publish(Point(x=nx, y=ny, z=1.0))
        if not self._target_visible:
            self._publish_behavior("notice")
            self._target_visible = True
        return True

    def _handle_object_tracking(self, blocks):
        if not blocks:
            return False
        nx, ny = self._normalize(blocks[0].x, blocks[0].y)
        self.pub_face.publish(Point(x=nx, y=ny, z=1.0))
        return True

    def _handle_object_recognition(self, blocks):
        if not blocks:
            return False
        target = blocks[0]
        nx, ny = self._normalize(target.x, target.y)
        self.pub_face.publish(Point(x=nx, y=ny, z=1.0))
        m = OBJECT_ID_MAP.get(target.ID)
        if m and not self._target_visible:
            self._publish_behavior(m["behavior"])
            self._set_iris(*m["iris"])
        return True

    def _handle_color_recognition(self, blocks):
        if not blocks:
            return False
        c = COLOR_ID_MAP.get(blocks[0].ID)
        if c:
            self._set_iris(*c)
        nx, ny = self._normalize(blocks[0].x, blocks[0].y)
        self.pub_face.publish(Point(x=nx, y=ny, z=1.0))
        return True

    def _handle_tag_recognition(self, blocks):
        if not blocks:
            return False
        m = OBJECT_ID_MAP.get(blocks[0].ID)
        if m:
            self._publish_behavior(m["behavior"])
        return True

    def _handle_line_tracking(self, arrows):
        if not arrows:
            return False
        a = arrows[0]
        dx = (a.x2 - a.x1) / HUSKY_W
        dy = (a.y2 - a.y1) / HUSKY_H
        self.pub_face.publish(Point(x=dx, y=dy, z=1.0))
        return True

    # -----------------------------------------------------------------------
    # Loop principal
    # -----------------------------------------------------------------------
    def run(self):
        rate = rospy.Rate(self.rate_hz)
        handler_map = {
            0: self._handle_face_recognition,
            1: self._handle_object_tracking,
            2: self._handle_object_recognition,
            3: self._handle_line_tracking,
            4: self._handle_color_recognition,
            5: self._handle_tag_recognition,
            6: self._handle_emotion_recognition,
        }
        handler = handler_map.get(self.hl_mode, self._handle_emotion_recognition)
        rospy.loginfo("HuskyLens bridge en marcha.")

        while not rospy.is_shutdown():
            detected = False
            try:
                data = self._hl.arrows() if self.hl_mode == 3 else self._hl.blocks()
                if data:
                    detected = handler(data)
            except Exception as e:
                rospy.logwarn_throttle(5.0, f"HuskyLens read error: {e}")

            if detected:
                self._last_seen = time.time()
            else:
                if self._target_visible and (time.time() - self._last_seen) > self.lost_timeout:
                    rospy.loginfo("Objetivo perdido → centro + neutral")
                    self._publish_behavior("look_center")
                    self.pub_face.publish(Point(x=0.0, y=0.0, z=0.0))
                    self._set_iris(60, 120, 200)   # iris azul por defecto
                    self._target_visible    = False
                    self._current_emotion   = None
                    self._last_emotion_sent = None

            rate.sleep()


def main():
    try:
        HuskyLensBridge().run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
