#!/usr/bin/env python3
"""
ros_node.py — Nodo ROS 2 para los ojos del robot
Migrado de ROS 1 (rospy) a ROS 2 Jazzy (rclpy).

Suscripciones:
  /robot_eyes/behavior      (std_msgs/String)    — nombre de comportamiento
  /robot_eyes/gaze          (geometry_msgs/Point) — dirección mirada [-1,1]
  /robot_eyes/emotion       (std_msgs/String)    — nombre de emoción
  /robot_eyes/face_position (geometry_msgs/Point) — posición cara normalizada
  /robot_eyes/pupil_size    (std_msgs/Float32)   — dilatación pupila [0,1]
  /robot_eyes/iris_color    (std_msgs/ColorRGBA) — color del iris

Publicaciones:
  /robot_eyes/state  (std_msgs/String) — comportamiento activo
  /robot_eyes/ready  (std_msgs/Bool)   — display inicializado

Servicios:
  /robot_eyes/play_behavior (std_srvs/Trigger) — ejecutar blink de demo

Parámetros:
  fps           (int,  30)    — Hz de render
  sim_mode      (bool, false) — sin pantallas reales
  auto_idle     (bool, true)  — comportamientos idle aleatorios
  iris_color    (list)        — [R,G,B] color iris por defecto
  background    (list)        — [R,G,B] color fondo
  display_config (dict)       — configuración SPI/GPIO
"""

import time
import threading
import random

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Bool, Float32, ColorRGBA
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import Point
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print('[WARN] rclpy not found — running in standalone simulation mode.')

from .eye_renderer import EyeRenderer, EyeState
from .animation_engine import AnimationEngine
from .behaviors import BehaviorLibrary
from .display_driver import DualDisplayController

# ── Mapas de nombre → factory ─────────────────────────────────────────────────
BEHAVIOR_MAP = {
    'blink':        BehaviorLibrary.blink,
    'double_blink': BehaviorLibrary.double_blink,
    'slow_blink':   BehaviorLibrary.slow_blink,
    'wink_right':   lambda: BehaviorLibrary.wink('right'),
    'wink_left':    lambda: BehaviorLibrary.wink('left'),
    'look_left':    BehaviorLibrary.look_left,
    'look_right':   BehaviorLibrary.look_right,
    'look_up':      BehaviorLibrary.look_up,
    'look_down':    BehaviorLibrary.look_down,
    'look_center':  BehaviorLibrary.look_center,
    'scan':         BehaviorLibrary.scan_horizontal,
    'thinking':     BehaviorLibrary.thinking,
    'roll_eyes':    BehaviorLibrary.roll_eyes,
    'dizzy':        BehaviorLibrary.dizzy,
    'look_away':    BehaviorLibrary.look_away_shy,
    'happy':        BehaviorLibrary.happy,
    'sad':          BehaviorLibrary.sad,
    'surprised':    BehaviorLibrary.surprised,
    'angry':        BehaviorLibrary.angry,
    'suspicious':   BehaviorLibrary.suspicious,
    'tired':        BehaviorLibrary.tired,
    'love':         BehaviorLibrary.love,
    'confused':     BehaviorLibrary.confused,
    'fall_asleep':  BehaviorLibrary.fall_asleep,
    'wake_up':      BehaviorLibrary.wake_up,
    'sleeping':     BehaviorLibrary.sleeping_loop,
    'dilate':       BehaviorLibrary.pupil_dilate,
    'random':       BehaviorLibrary.random_behavior,
}

EMOTION_MAP = {
    'happy':      BehaviorLibrary.happy,
    'sad':        BehaviorLibrary.sad,
    'surprised':  BehaviorLibrary.surprised,
    'angry':      BehaviorLibrary.angry,
    'neutral':    BehaviorLibrary.look_center,
    'love':       BehaviorLibrary.love,
    'confused':   BehaviorLibrary.confused,
    'tired':      BehaviorLibrary.tired,
    'suspicious': BehaviorLibrary.suspicious,
    'bored':      BehaviorLibrary.tired,
}


# ── Nodo ROS 2 ────────────────────────────────────────────────────────────────
class RobotEyesNode(Node if HAS_ROS else object):

    def __init__(self):
        if HAS_ROS:
            super().__init__('robot_eyes_node')
            self._declare_params()
            fps        = self.get_parameter('fps').value
            sim_mode   = self.get_parameter('sim_mode').value
            auto_idle  = self.get_parameter('auto_idle').value
            bg_color   = self.get_parameter('background').value
            iris_color = self.get_parameter('iris_color').value
        else:
            fps        = 30
            sim_mode   = True
            auto_idle  = True
            bg_color   = [10, 10, 15]
            iris_color = [60, 120, 200]

        self._fps       = fps
        self._sim_mode  = sim_mode
        self._auto_idle = auto_idle

        # ── Core ────────────────────────────────────────────────────────────
        self._renderer = EyeRenderer(background_color=tuple(bg_color))
        self._engine   = AnimationEngine(fps=fps)
        self._engine._base_left.iris_color  = tuple(iris_color)
        self._engine._base_right.iris_color = tuple(iris_color)

        # ── Display ──────────────────────────────────────────────────────────
        self._display = None
        self._ready   = False
        if not sim_mode:
            if HAS_ROS:
                display_cfg = self.get_parameter('display_config').value or {}
            else:
                display_cfg = {}
            self._display = DualDisplayController(config=display_cfg)
            try:
                self._display.begin()
                self._ready = True
            except Exception as e:
                self._log_error(f'Display init failed: {e}')
        else:
            self._ready = True
            self._log_info('Running in simulation mode — no real displays.')

        # ── Publicadores ─────────────────────────────────────────────────────
        if HAS_ROS:
            self._pub_state = self.create_publisher(String, '/robot_eyes/state', 1)
            self._pub_ready = self.create_publisher(Bool,   '/robot_eyes/ready', 1)

            ready_msg = Bool()
            ready_msg.data = self._ready
            self._pub_ready.publish(ready_msg)

            # Suscriptores
            self.create_subscription(String,    '/robot_eyes/behavior',      self._cb_behavior,  10)
            self.create_subscription(Point,     '/robot_eyes/gaze',          self._cb_gaze,      10)
            self.create_subscription(String,    '/robot_eyes/emotion',       self._cb_emotion,   10)
            self.create_subscription(Point,     '/robot_eyes/face_position', self._cb_face,      10)
            self.create_subscription(Float32,   '/robot_eyes/pupil_size',    self._cb_pupil,     10)
            self.create_subscription(ColorRGBA, '/robot_eyes/iris_color',    self._cb_iris_color, 10)

            # Servicio
            self.create_service(Trigger, '/robot_eyes/play_behavior', self._srv_play)

        # ── Engine + display thread ───────────────────────────────────────────
        self._idle_last     = time.time()
        self._idle_interval = random.uniform(4.0, 10.0)
        self._running = True
        self._engine.start()
        self._display_thread = threading.Thread(target=self._display_loop, daemon=True)
        self._display_thread.start()

        self._log_info('RobotEyes node ready.')

    # ── Declaración de parámetros ROS 2 ──────────────────────────────────────
    def _declare_params(self):
        from rclpy.parameter import Parameter
        self.declare_parameter('fps',            30)
        self.declare_parameter('sim_mode',       False)
        self.declare_parameter('auto_idle',      True)
        self.declare_parameter('iris_color',     [60, 120, 200])
        self.declare_parameter('background',     [10, 10, 15])
        self.declare_parameter('display_config', '')

    # ── Logging helpers ───────────────────────────────────────────────────────
    def _log_info(self, msg):
        if HAS_ROS:
            self.get_logger().info(msg)
        else:
            print(f'[INFO] {msg}')

    def _log_warn(self, msg):
        if HAS_ROS:
            self.get_logger().warn(msg)
        else:
            print(f'[WARN] {msg}')

    def _log_error(self, msg):
        if HAS_ROS:
            self.get_logger().error(msg)
        else:
            print(f'[ERR]  {msg}')

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _cb_behavior(self, msg: 'String'):
        self._play_by_name(msg.data.strip().lower())

    def _cb_gaze(self, msg: 'Point'):
        anim = BehaviorLibrary.look_at(gaze_x=float(msg.x), gaze_y=float(msg.y), duration=0.15)
        self._engine.play(anim)

    def _cb_emotion(self, msg: 'String'):
        name = msg.data.strip().lower()
        factory = EMOTION_MAP.get(name)
        if factory:
            self._engine.play(factory())
        else:
            self._log_warn(f'Unknown emotion: {name}')

    def _cb_face(self, msg: 'Point'):
        gx =  float(msg.x) * 0.7
        gy = -float(msg.y) * 0.5
        self._engine.set_base_gaze(gx, gy)

    def _cb_pupil(self, msg: 'Float32'):
        self._engine._base_left.pupil_size  = float(msg.data)
        self._engine._base_right.pupil_size = float(msg.data)

    def _cb_iris_color(self, msg: 'ColorRGBA'):
        color = (int(msg.r * 255), int(msg.g * 255), int(msg.b * 255))
        self._engine._base_left.iris_color  = color
        self._engine._base_right.iris_color = color

    def _srv_play(self, _req, response):
        self._engine.play(BehaviorLibrary.blink())
        response.success = True
        response.message = 'Playing blink'
        return response

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _play_by_name(self, name: str):
        factory = BEHAVIOR_MAP.get(name)
        if factory:
            self._engine.play(factory())
            if HAS_ROS:
                msg = String()
                msg.data = name
                self._pub_state.publish(msg)
        else:
            self._log_warn(
                f'Unknown behavior: "{name}". Available: {list(BEHAVIOR_MAP.keys())}')

    def _display_loop(self):
        dt = 1.0 / self._fps
        while self._running:
            t0 = time.time()

            left_state, right_state = self._engine.get_states()
            left_img  = self._renderer.render(left_state)
            right_img = self._re