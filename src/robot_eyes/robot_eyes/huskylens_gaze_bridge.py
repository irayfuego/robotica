#!/usr/bin/env python3
"""huskylens_gaze_bridge.py -- HuskyLens 2 I2C gaze (+emotion) bridge.

Two operating modes, selected by the 'algo' ROS parameter:

  algo=3  (Object Tracking) -- GAZE ONLY
      Eyes follow a specific learned object.
      To start tracking: call the ROS service /huskylens/learn
        ros2 service call /huskylens/learn std_srvs/Trigger
      To forget:         call /huskylens/forget

  algo=13 (Face Emotion Recognition) -- GAZE + EMOTION
      Eyes automatically follow any detected face AND publish the emotion.
      No learning required.
      Publishes /robot_eyes/gaze AND /robot_eyes/emotion.

Protocol: HuskyLens V2  (NOT legacy XIAO/V1)
Frame:    0x55 0xAA CMD AlgoID DataLen [Data...] Checksum
Checksum  = sum(preceding bytes) & 0xFF

Wiring (I2C bus 1):
  SDA -> GPIO 2 / Pin 3
  SCL -> GPIO 3 / Pin 5
  Device menu: protocol = I2C, address = 0x50
"""

import struct
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_srvs.srv import Trigger

# HuskyLens V2 commands
_CMD_KNOCK        = 0x00
_CMD_GET_RESULT   = 0x01
_CMD_LEARN        = 0x02   # learn current detection, returns id in RETURN_ARGS
_CMD_FORGET       = 0x06   # forget all learned objects
_CMD_SET_ALGO     = 0x0A
_CMD_RETURN_ARGS  = 0x1A
_CMD_RETURN_INFO  = 0x1B
_CMD_RETURN_BLOCK = 0x1C

_ALGO_ANY            = 0
_ALGO_OBJ_TRACKING   = 3    # Object Tracking
_ALGO_FACE_EMOTION   = 13   # Face Emotion Recognition

_HL_W = 640
_HL_H = 480

_DEFAULT_EMOTION_MAP = {
    1: "neutral",
    2: "happy",
    3: "sad",
    4: "surprised",
    5: "angry",
    6: "confused",
    7: "suspicious",
}


# ---------------------------------------------------------------------------
# Frame helpers
# ---------------------------------------------------------------------------

def _build(cmd, algo, data=b''):
    frame = bytes([0x55, 0xAA, cmd, algo, len(data)]) + data
    return frame + bytes([sum(frame) & 0xFF])


def _find_frame(buf, expected_cmd):
    i = 0
    while i < len(buf) - 5:
        if buf[i] != 0x55 or buf[i + 1] != 0xAA:
            i += 1
            continue
        dlen = buf[i + 4]
        frame_end = i + 5 + dlen
        if frame_end >= len(buf):
            break
        if buf[frame_end] != (sum(buf[i:frame_end]) & 0xFF):
            i += 1
            continue
        cmd  = buf[i + 2]
        algo = buf[i + 3]
        data = buf[i + 5:frame_end]
        if cmd == expected_cmd:
            return cmd, algo, data, frame_end + 1
        i = frame_end + 1
    return None


# ---------------------------------------------------------------------------
# Low-level I2C driver
# ---------------------------------------------------------------------------

class _HuskyLensI2C:
    _ADDR    = 0x50
    _CHUNK   = 32
    _READ_SZ = 16
    _RETRY   = 3
    _TIMEOUT = 1.5

    def __init__(self, bus_num=1):
        from smbus2 import SMBus, i2c_msg
        self._bus = SMBus(bus_num)
        self._msg = i2c_msg

    def _write(self, frame):
        for off in range(0, len(frame), self._CHUNK):
            w = self._msg.write(self._ADDR, list(frame[off:off + self._CHUNK]))
            self._bus.i2c_rdwr(w)

    def _wait_for(self, expected_cmd, timeout=None):
        deadline = time.time() + (timeout or self._TIMEOUT)
        buf = bytearray()
        seen_non_ff = False
        while time.time() < deadline:
            try:
                r = self._msg.read(self._ADDR, self._READ_SZ)
                self._bus.i2c_rdwr(r)
                chunk = bytes(r)
            except Exception:
                time.sleep(0.005)
                continue
            if all(b == 0xFF for b in chunk) and not seen_non_ff:
                time.sleep(0.005)
                continue
            seen_non_ff = True
            buf += chunk
            result = _find_frame(bytes(buf), expected_cmd)
            if result is not None:
                return result[0], result[1], result[2]
        return None

    def knock(self):
        # Any RETURN_ARGS response = device alive.
        # retValue is 0 on first boot, 1 when already running an algorithm.
        frame = _build(_CMD_KNOCK, _ALGO_ANY, bytes(10))
        for _ in range(self._RETRY):
            try:
                self._write(frame)
                time.sleep(0.01)
                resp = self._wait_for(_CMD_RETURN_ARGS)
                if resp is not None:
                    return True
            except Exception:
                pass
            time.sleep(0.1)
        return False

    def switch_algorithm(self, algo):
        # 0.2 s sleep is required -- heavy ML models take time to unload/load.
        frame = _build(_CMD_SET_ALGO, _ALGO_ANY, bytes([algo]) + bytes(9))
        for _ in range(self._RETRY):
            try:
                self._write(frame)
                time.sleep(0.2)
                resp = self._wait_for(_CMD_RETURN_ARGS, timeout=8.0)
                if resp and len(resp[2]) >= 2 and resp[2][1] == 0:
                    return True
            except Exception:
                pass
            time.sleep(0.2)
        return False

    def learn(self, algo):
        """Send learn command. Returns learned object ID, or 0 on failure."""
        frame = _build(_CMD_LEARN, algo)
        try:
            self._write(frame)
            time.sleep(0.05)
            resp = self._wait_for(_CMD_RETURN_ARGS, timeout=2.0)
            if resp and len(resp[2]) >= 1:
                return resp[2][0]  # learned ID
        except Exception:
            pass
        return 0

    def forget(self, algo):
        """Forget all learned objects for this algorithm."""
        frame = _build(_CMD_FORGET, algo)
        try:
            self._write(frame)
            time.sleep(0.05)
            resp = self._wait_for(_CMD_RETURN_ARGS, timeout=2.0)
            if resp is not None:
                return True
        except Exception:
            pass
        return False

    def request_blocks(self, algo):
        frame = _build(_CMD_GET_RESULT, algo)
        try:
            self._write(frame)
        except Exception:
            return []
        info = self._wait_for(_CMD_RETURN_INFO, timeout=0.4)
        if info is None or len(info[2]) < 10:
            return []
        total_blocks = struct.unpack_from('<H', info[2], 6)[0]
        if total_blocks == 0:
            return []
        blocks = []
        for _ in range(min(total_blocks, 5)):
            blk = self._wait_for(_CMD_RETURN_BLOCK, timeout=0.4)
            if blk is None or len(blk[2]) < 10:
                break
            d = blk[2]
            blocks.append((
                struct.unpack_from('<h', d, 2)[0],   # xCenter
                struct.unpack_from('<h', d, 4)[0],   # yCenter
                struct.unpack_from('<H', d, 6)[0],   # width
                struct.unpack_from('<H', d, 8)[0],   # height
                d[0],                                 # ID or emotion class
            ))
        return blocks

    def cleanup(self):
        try:
            self._bus.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class HuskyLensGazeBridge(Node):
    """
    algo=3  -> Object Tracking: gaze only (/robot_eyes/gaze).
               Call /huskylens/learn to start tracking what the camera sees.
    algo=13 -> Face Emotion: gaze + emotion (/robot_eyes/gaze + /robot_eyes/emotion).
    """

    _ALPHA = 0.35

    def __init__(self):
        super().__init__('huskylens_gaze_bridge')

        self.declare_parameter('i2c_bus',    1)
        self.declare_parameter('hz',         20)
        self.declare_parameter('algo',       _ALGO_OBJ_TRACKING)
        self.declare_parameter('hl_width',   _HL_W)
        self.declare_parameter('hl_height',  _HL_H)
        self.declare_parameter('emotion_map', '')
        # Imitar la emocion de las caras detectadas. Por defecto NO: el robot
        # sigue la cara con la mirada pero no copia la emocion (resultaba
        # invasivo y disparaba 'angry'/rojo). Activable en marcha por voz via
        # /robot/gaze_control con 'emotion_on' / 'emotion_off'.
        self.declare_parameter('publish_emotion', False)
        # Seguimiento de mirada: ganancia (amplifica el movimiento del iris para
        # que el seguimiento se note) e inversion de cada eje, por si la camara
        # esta montada/orientada de forma que el eje sale al reves.
        self.declare_parameter('gaze_gain',     1.8)
        self.declare_parameter('gaze_invert_x', False)
        self.declare_parameter('gaze_invert_y', False)

        bus_num        = self.get_parameter('i2c_bus').value
        hz             = self.get_parameter('hz').value
        self._algo     = self.get_parameter('algo').value
        self._hlw      = self.get_parameter('hl_width').value
        self._hlh      = self.get_parameter('hl_height').value
        emap_str       = self.get_parameter('emotion_map').value
        self._publish_emotion = bool(self.get_parameter('publish_emotion').value)
        self._gain  = float(self.get_parameter('gaze_gain').value)
        self._inv_x = bool(self.get_parameter('gaze_invert_x').value)
        self._inv_y = bool(self.get_parameter('gaze_invert_y').value)

        self._emotion_map = dict(_DEFAULT_EMOTION_MAP)
        if emap_str:
            for entry in emap_str.split(','):
                try:
                    k, v = entry.strip().split(':')
                    self._emotion_map[int(k.strip())] = v.strip()
                except Exception:
                    pass

        self._pub_gaze    = self.create_publisher(Point,  '/robot_eyes/gaze',    10)
        self._pub_emotion = self.create_publisher(String, '/robot_eyes/emotion', 10)

        # ROS services for software learn / forget (no physical button needed)
        self.create_service(Trigger, '/huskylens/learn',  self._srv_learn)
        self.create_service(Trigger, '/huskylens/forget', self._srv_forget)

        # Control de la mirada por voz: 'follow' (sigue caras con la mirada,
        # por defecto) | 'rest' (no controla los ojos -> idle) |
        # 'emotion_on' / 'emotion_off' (imitar emociones).
        self._gaze_enabled = True
        self.create_subscription(String, '/robot/gaze_control',
                                 self._cb_gaze_control, 10)

        self._gx              = 0.0
        self._gy              = 0.0
        self._last_emotion    = None
        self._last_face_time  = 0.0   # wall-clock time of last face/block detection
        self._hl              = None

        mode_label = {
            _ALGO_OBJ_TRACKING: 'Object Tracking  -- call /huskylens/learn to track object',
            _ALGO_FACE_EMOTION:  'Face Emotion Recognition (automatic)',
        }.get(self._algo, 'algo %d' % self._algo)

        try:
            self._hl = _HuskyLensI2C(bus_num)
            if not self._hl.knock():
                raise RuntimeError('knock failed -- check I2C wiring and protocol=I2C on device')
            if not self._hl.switch_algorithm(self._algo):
                raise RuntimeError('could not switch to algo %d' % self._algo)
            self.get_logger().info(
                'HuskyLens 2 ready  bus=%d  addr=0x50  algo=%d  mode: %s'
                % (bus_num, self._algo, mode_label))
        except Exception as e:
            self.get_logger().error('HuskyLens init failed: %s' % e)

        self.create_timer(1.0 / hz, self._poll)

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _srv_learn(self, _req, response):
        if self._hl is None:
            response.success = False
            response.message = 'HuskyLens not initialised'
            return response
        obj_id = self._hl.learn(self._algo)
        if obj_id:
            self.get_logger().info('Learned object ID=%d' % obj_id)
            response.success = True
            response.message = 'Learned object ID=%d' % obj_id
        else:
            self.get_logger().warn('Learn failed -- nothing to learn in frame?')
            response.success = False
            response.message = 'Learn failed -- point camera at target first'
        return response

    def _srv_forget(self, _req, response):
        if self._hl is None:
            response.success = False
            response.message = 'HuskyLens not initialised'
            return response
        ok = self._hl.forget(self._algo)
        self.get_logger().info('Forget: %s' % ('OK' if ok else 'FAILED'))
        response.success = ok
        response.message = 'Forgot all learned objects' if ok else 'Forget failed'
        return response

    # ------------------------------------------------------------------
    # Gaze control (by voice)
    # ------------------------------------------------------------------

    def _cb_gaze_control(self, msg):
        m = msg.data.strip().lower()
        if m in ('follow', 'on', 'seguir', 'face'):
            self._gaze_enabled = True
            self._switch_algo_async(_ALGO_FACE_EMOTION)    # seguir caras
            self.get_logger().info('Mirada por camara: SEGUIR (caras)')
        elif m in ('track', 'track_object', 'objeto'):
            self._gaze_enabled = True
            self._switch_algo_async(_ALGO_OBJ_TRACKING, learn_after=True)
            self.get_logger().info('Mirada por camara: SEGUIR OBJETO')
        elif m in ('rest', 'off', 'reposo', 'descansa'):
            self._gaze_enabled = False
            self.get_logger().info('Mirada por camara: REPOSO')
        elif m == 'emotion_on':
            self._publish_emotion = True
            self._gaze_enabled = True
            self._switch_algo_async(_ALGO_FACE_EMOTION)
            self.get_logger().info('Mirada por camara: imitar emociones ON')
        elif m == 'emotion_off':
            self._publish_emotion = False
            self.get_logger().info('Mirada por camara: imitar emociones OFF')

    def _switch_algo_async(self, algo, learn_after=False):
        """Cambia el algoritmo de la HuskyLens en un hilo (el switch puede
        bloquear varios segundos) y, opcionalmente, aprende el objeto que tiene
        delante (Object Tracking: aprende lo que ve en el centro para seguirlo)."""
        def _work():
            try:
                if self._hl is None:
                    return
                if self._algo != algo:
                    if not self._hl.switch_algorithm(algo):
                        self.get_logger().warn('No se pudo cambiar a algo %d' % algo)
                        return
                    self._algo = algo
                    self.get_logger().info('Algoritmo de camara -> %d' % algo)
                if learn_after:
                    time.sleep(0.3)
                    oid = self._hl.learn(algo)
                    self.get_logger().info('Objeto aprendido id=%d (a seguir)' % oid)
            except Exception as e:
                self.get_logger().warn('cambio de algoritmo fallo: %s' % e)
        threading.Thread(target=_work, daemon=True).start()

    # ------------------------------------------------------------------
    # Poll timer
    # ------------------------------------------------------------------

    def _poll(self):
        if self._hl is None:
            return
        try:
            blocks = self._hl.request_blocks(self._algo)
        except Exception as e:
            self.get_logger().warn(
                'HuskyLens read error: %s' % e, throttle_duration_sec=3.0)
            return

        now = time.time()

        if not blocks:
            # algo=13: if no face seen for 2 s, publish neutral to reset eyes
            if (self._algo == _ALGO_FACE_EMOTION
                    and self._publish_emotion
                    and self._last_emotion is not None
                    and self._last_face_time > 0
                    and now - self._last_face_time > 2.0):
                self._last_emotion = None
                emo_msg = String()
                emo_msg.data = 'neutral'
                self._pub_emotion.publish(emo_msg)
                self.get_logger().info('No face -- reset emotion to neutral')
            return

        self._last_face_time = now
        xc, yc, _w, _h, block_id = blocks[0]

        # --- Gaze: posicion de la cara en el frame -> mirada [-1,1] ---
        # cx/cy en [-1,1] (centro del frame = 0). El eje vertical YA no se
        # niega: la convencion del renderer (gaze_y+ = mirar abajo) coincide con
        # yc+ = parte baja del frame. La ganancia amplifica el movimiento para
        # que el seguimiento se vea; cada eje es invertible por parametro.
        cx = (xc / (self._hlw / 2.0)) - 1.0
        cy = (yc / (self._hlh / 2.0)) - 1.0
        if self._inv_x:
            cx = -cx
        if self._inv_y:
            cy = -cy
        raw_x = max(-1.0, min(1.0, cx * self._gain))
        raw_y = max(-1.0, min(1.0, cy * self._gain))
        a        = self._ALPHA
        self._gx = a * raw_x + (1.0 - a) * self._gx
        self._gy = a * raw_y + (1.0 - a) * self._gy

        # Solo movemos los ojos con la camara en modo 'follow'. En 'rest' se
        # deja que el robot_eyes_node haga su idle (parpadeos/miradas).
        if self._gaze_enabled:
            gaze_msg = Point()
            gaze_msg.x = float(self._gx)
            gaze_msg.y = float(self._gy)
            self._pub_gaze.publish(gaze_msg)

        # --- Emotion (algo=13, solo si esta activada la imitacion) ---
        if self._algo == _ALGO_FACE_EMOTION and self._publish_emotion:
            emotion = self._emotion_map.get(block_id)
            if emotion and emotion != self._last_emotion:
                self._last_emotion = emotion
                self.get_logger().info(
                    'Emotion: %s  (id=%d  face=(%d,%d))' % (emotion, block_id, xc, yc))
                emo_msg = String()
                emo_msg.data = emotion
                self._pub_emotion.publish(emo_msg)
