#!/usr/bin/env python3
"""
ros_node.py -- ROS 2 node for robot eyes.

Subscriptions:
  /robot_eyes/behavior      (std_msgs/String)     -- behavior name
  /robot_eyes/gaze          (geometry_msgs/Point) -- persistent gaze target [-1,1]
                                                     (eyes ease toward it smoothly)
  /robot_eyes/emotion       (std_msgs/String)     -- emotion name (from HuskyLens)
  /robot_eyes/face_position (geometry_msgs/Point) -- normalised face position
  /robot_eyes/pupil_size    (std_msgs/Float32)    -- pupil dilation [0,1]
  /robot_eyes/iris_color    (std_msgs/ColorRGBA)  -- iris color

Publications:
  /robot_eyes/state  (std_msgs/String) -- active behavior name
  /robot_eyes/ready  (std_msgs/Bool)   -- display initialised

Services:
  /robot_eyes/play_behavior (std_srvs/Trigger) -- demo blink

Parameters:
  fps, sim_mode, auto_idle, iris_color, background, display_config,
  emotion_timeout
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
    print('[WARN] rclpy not found -- running in simulation mode.')

from .eye_renderer import EyeRenderer, EyeState
from .animation_engine import AnimationEngine, Animation, Keyframe, EasingType
from .behaviors import BehaviorLibrary, BEHAVIOR_MAP, EMOTION_STATES
from .display_driver import DualDisplayController

# ---- Emotion name -> animation factory (ALL HuskyLens algo=13 outputs) ------
EMOTION_MAP = {
    'neutral':    BehaviorLibrary.neutral,
    'happy':      BehaviorLibrary.happy,
    'sad':        BehaviorLibrary.sad,
    'surprised':  BehaviorLibrary.surprised,
    'angry':      BehaviorLibrary.angry,
    'confused':   BehaviorLibrary.confused,
    'suspicious': BehaviorLibrary.suspicious,
    'tired':      BehaviorLibrary.tired,
    'love':       BehaviorLibrary.love,
    'sleeping':   BehaviorLibrary.sleeping_loop,
    'bored':      BehaviorLibrary.tired,
}

# Default seconds without an emotion update before auto-resetting to neutral
_EMOTION_TIMEOUT = 8.0


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
            emo_tout   = self.get_parameter('emotion_timeout').value
        else:
            fps        = 30
            sim_mode   = True
            auto_idle  = True
            bg_color   = [10, 10, 15]
            iris_color = [60, 120, 200]
            emo_tout   = _EMOTION_TIMEOUT

        self._fps             = fps
        self._sim_mode        = sim_mode
        self._auto_idle       = auto_idle
        self._emotion_timeout = emo_tout

        self._renderer = EyeRenderer(background_color=tuple(bg_color))
        self._engine   = AnimationEngine(fps=fps)
        self._engine._base_left.iris_color  = tuple(iris_color)
        self._engine._base_right.iris_color = tuple(iris_color)

        # Neutral expression keeps the configured/last-set iris color
        # (a plain EyeState() would reset it to the hardcoded default blue)
        self._neutral_state = EyeState(iris_color=tuple(iris_color))

        # Track when the last emotion arrived to auto-reset to neutral
        self._last_emotion_time = 0.0
        self._current_emotion   = 'neutral'

        # Display
        self._display = None
        self._ready   = False
        if not sim_mode:
            display_cfg = self.get_parameter('display_config').value or {} if HAS_ROS else {}
            self._display = DualDisplayController(config=display_cfg)
            try:
                self._display.begin()
                self._ready = True
            except Exception as e:
                self._log_error('Display init failed: %s' % e)
        else:
            self._ready = True
            self._log_info('Running in simulation mode -- no real displays.')

        if HAS_ROS:
            self._pub_state = self.create_publisher(String, '/robot_eyes/state', 1)
            self._pub_ready = self.create_publisher(Bool,   '/robot_eyes/ready', 1)

            ready_msg = Bool(); ready_msg.data = self._ready
            self._pub_ready.publish(ready_msg)

            self.create_subscription(String,    '/robot_eyes/behavior',      self._cb_behavior,   10)
            self.create_subscription(Point,     '/robot_eyes/gaze',          self._cb_gaze,       10)
            self.create_subscription(String,    '/robot_eyes/emotion',       self._cb_emotion,    10)
            self.create_subscription(Point,     '/robot_eyes/face_position', self._cb_face,       10)
            self.create_subscription(Float32,   '/robot_eyes/pupil_size',    self._cb_pupil,      10)
            self.create_subscription(ColorRGBA, '/robot_eyes/iris_color',    self._cb_iris_color, 10)

            self.create_service(Trigger, '/robot_eyes/play_behavior', self._srv_play)

        self._idle_last     = time.time()
        self._idle_interval = random.uniform(6.0, 15.0)
        self._running       = True
        self._engine.start()
        self._display_thread = threading.Thread(target=self._display_loop, daemon=True)
        self._display_thread.start()

        self._log_info('RobotEyes node ready.')

    # ---------------------------------------------------------------- params

    def _declare_params(self):
        self.declare_parameter('fps',             30)
        self.declare_parameter('sim_mode',        False)
        self.declare_parameter('auto_idle',       True)
        self.declare_parameter('iris_color',      [60, 120, 200])
        self.declare_parameter('background',      [10, 10, 15])
        self.declare_parameter('display_config',  '')
        self.declare_parameter('emotion_timeout', _EMOTION_TIMEOUT)

    # --------------------------------------------------------------- logging

    def _log_info(self, msg):
        (self.get_logger().info if HAS_ROS else lambda m: print('[INFO]', m))(msg)

    def _log_warn(self, msg):
        (self.get_logger().warn if HAS_ROS else lambda m: print('[WARN]', m))(msg)

    def _log_error(self, msg):
        (self.get_logger().error if HAS_ROS else lambda m: print('[ERR] ', m))(msg)

    # -------------------------------------------------------------- callbacks

    def _cb_behavior(self, msg):
        self._play_by_name(msg.data.strip().lower())

    def _cb_gaze(self, msg):
        # Persistent gaze target; the engine eases toward it (smooth pursuit)
        self._engine.set_base_gaze(float(msg.x), float(msg.y))

    def _cb_emotion(self, msg):
        name = msg.data.strip().lower()
        factory = EMOTION_MAP.get(name)
        if not factory:
            self._log_warn('Unknown emotion: %s' % name)
            return

        # Play the transition animation and persist the expression into the
        # base state so it survives blinks and idle glances
        if name != 'neutral':
            self._engine.play(factory())
            target_state = EMOTION_STATES.get(name)
            if target_state:
                self._engine.set_base_expression(target_state)
        else:
            # Reset to neutral, keeping the configured iris color
            self._engine.play(self._make_neutral_anim())
            self._engine.set_base_expression(self._neutral_state)

        self._current_emotion   = name
        self._last_emotion_time = time.time()

    def _cb_face(self, msg):
        gx =  float(msg.x) * 0.7
        gy = -float(msg.y) * 0.5
        self._engine.set_base_gaze(gx, gy)

    def _cb_pupil(self, msg):
        v = float(msg.data)
        self._engine._base_left.pupil_size  = v
        self._engine._base_right.pupil_size = v

    def _cb_iris_color(self, msg):
        color = (int(msg.r * 255), int(msg.g * 255), int(msg.b * 255))
        self._engine._base_left.iris_color  = color
        self._engine._base_right.iris_color = color
        # The externally-set color becomes the new neutral color too
        self._neutral_state.iris_color = color

    def _srv_play(self, _req, response):
        self._engine.play(BehaviorLibrary.blink())
        response.success = True
        response.message = 'Playing blink'
        return response

    # --------------------------------------------------------------- helpers

    # Expression-only channels: relax the face without recentering the gaze
    _EXPR_CHANNELS = frozenset({'lids', 'pupil', 'iris', 'squint', 'eyebrow'})

    def _make_neutral_anim(self):
        """Neutral transition built from the node's own neutral state,
        so it never flashes the hardcoded default iris color."""
        kf = [Keyframe(self._neutral_state, 0.35, EasingType.EASE_IN_OUT)]
        return Animation('neutral', left_keyframes=kf,
                         channels=self._EXPR_CHANNELS)

    def _play_by_name(self, name):
        factory = BEHAVIOR_MAP.get(name)
        if factory:
            anim = self._make_neutral_anim() if name == 'neutral' else factory()
            self._engine.play(anim)
            if HAS_ROS:
                msg = String(); msg.data = name
                self._pub_state.publish(msg)
        else:
            self._log_warn('Unknown behavior: "%s"' % name)

    # ------------------------------------------------------------- main loop

    def _display_loop(self):
        dt = 1.0 / self._fps
        while self._running:
            t0  = time.time()
            now = t0

            left_state, right_state = self._engine.get_states()
            left_img  = self._renderer.render(left_state)
            right_img = self._renderer.render(right_state)

            if self._display and self._ready:
                try:
                    self._display.update(left_img, right_img)
                except Exception as e:
                    print('[ERR] Display update: %s' % e)

            if self._auto_idle:
                # Idle gaze movements (separate from blink; blink handled in engine)
                if now - self._idle_last > self._idle_interval:
                    self._idle_last     = now
                    self._idle_interval = random.uniform(6.0, 18.0)
                    if self._engine._current_anim is None:
                        self._engine.play(BehaviorLibrary.random_behavior())

                # Emotion timeout: without updates, fade back to neutral
                if (self._current_emotion != 'neutral'
                        and self._last_emotion_time > 0
                        and now - self._last_emotion_time > self._emotion_timeout):
                    self._current_emotion = 'neutral'
                    self._engine.set_base_expression(self._neutral_state)
                    self._engine.play(self._make_neutral_anim())

            elapsed = time.time() - t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def shutdown(self):
        self._running = False
        self._engine.stop()
        if self._display:
            self._display.cleanup()
        self._log_info('RobotEyes shutdown complete.')


def main(args=None):
    if not HAS_ROS:
        node = RobotEyesNode()
        try:
            while True: time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        node.shutdown()
        return

    rclpy.init(args=args)
    node = RobotEyesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
