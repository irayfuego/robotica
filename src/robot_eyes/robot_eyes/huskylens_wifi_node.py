#!/usr/bin/env python3
"""
huskylens_wifi_node.py — Integración HuskyLens 2 vía Wi-Fi con ROS 2 + TTS

Publica los mismos topics que huskylens_node.py (I2C) pero usando HTTP.
Además publica en /tts/say cuando detecta caras o emociones.

Parámetros:
  host         (str)   192.168.1.32
  port         (int)   3000
  poll_rate    (float) 5.0  Hz
  algorithm    (int)   13   (Face Emotion Recognition por defecto)
  tts_enabled  (bool)  true
  frame_id     (str)   camera_link
"""

import json
import urllib.request
import urllib.error
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Point
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

# Mensajes TTS por emoción detectada
EMOTION_PHRASES = {
    'Happiness': ['¡Qué alegría verte!', '¡Me encanta tu sonrisa!', '¡Estás muy contento!'],
    'Neutral':   ['Hola, te veo por aquí.', 'Buenos días.'],
    'Sadness':   ['Pareces triste. ¿Puedo ayudarte?', 'Ánimo, todo irá bien.'],
    'Anger':     ['Veo que estás enfadado. Tranquilo.', 'Respira hondo.'],
    'Surprise':  ['¡Vaya sorpresa!', '¡Oh! ¿Te he sorprendido?'],
    'Fear':      ['No tengas miedo, estoy aquí.'],
    'Disgust':   ['Entiendo tu reacción.'],
}


class HuskyLensWifiNode(Node):

    def __init__(self):
        super().__init__('huskylens_wifi_node')

        self.declare_parameter('host',        '192.168.1.32')
        self.declare_parameter('port',        3000)
        self.declare_parameter('poll_rate',   5.0)
        self.declare_parameter('algorithm',   13)
        self.declare_parameter('tts_enabled', True)
        self.declare_parameter('frame_id',    'camera_link')

        self._host      = self.get_parameter('host').value
        self._port      = self.get_parameter('port').value
        self._rate      = self.get_parameter('poll_rate').value
        self._algo      = self.get_parameter('algorithm').value
        self._tts       = self.get_parameter('tts_enabled').value
        self._frame_id  = self.get_parameter('frame_id').value
        self._base_url  = f'http://{self._host}:{self._port}'

        # Publicadores (mismos topics que nodo I2C)
        self._det_pub   = self.create_publisher(Detection2DArray, 'huskylens/detections', 10)
        self._point_pub = self.create_publisher(Point,            'huskylens/tracked_object', 10)
        self._algo_pub  = self.create_publisher(String,           'huskylens/algorithm', 1)
        self._tts_pub   = self.create_publisher(String,           '/tts/say', 5)

        # Estado
        self._last_emotion = None
        self._emotion_idx  = {}  # para rotar frases
        self._det_count    = 0

        self.create_timer(1.0 / self._rate, self._poll)
        self.get_logger().info(
            f'huskylens_wifi_node → {self._base_url} algoritmo={self._algo} @ {self._rate}Hz')

    def _call(self, path: str, data: dict = None) -> dict:
        url = f'{self._base_url}{path}'
        body = json.dumps(data).encode() if data else None
        req  = urllib.request.Request(url, data=body,
               headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=2.0) as r:
            return json.loads(r.read())

    def _poll(self):
        try:
            result = self._call('/recognition/result',
                                {'algorithm': self._algo, 'operation': 'get_result'})
        except Exception as e:
            self.get_logger().warn(f'Error HTTP: {e}', throttle_duration_sec=10.0)
            return

        detections = result if isinstance(result, list) else []
        now = self.get_clock().now().to_msg()

        arr = Detection2DArray()
        arr.header.stamp    = now
        arr.header.frame_id = self._frame_id

        for det in detections:
            d = Detection2D()
            d.header = arr.header
            d.bbox.center.position.x = float(det.get('xCenter', 0))
            d.bbox.center.position.y = float(det.get('yCenter', 0))
            d.bbox.size_x = float(det.get('width', 0))
            d.bbox.size_y = float(det.get('height', 0))
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(det.get('name', det.get('content', '')))
            hyp.hypothesis.score    = 1.0
            d.results.append(hyp)
            arr.detections.append(d)

        self._det_pub.publish(arr)

        # Publicar punto del objeto principal
        if detections:
            primary = detections[0]
            pt = Point()
            pt.x = float(primary.get('xCenter', 0))
            pt.y = float(primary.get('yCenter', 0))
            self._point_pub.publish(pt)

            # TTS por emoción
            if self._tts:
                emotion = primary.get('name') or primary.get('content', '')
                if emotion and emotion != self._last_emotion:
                    self._last_emotion = emotion
                    phrases = EMOTION_PHRASES.get(emotion, [])
                    if phrases:
                        idx = self._emotion_idx.get(emotion, 0) % len(phrases)
                        self._emotion_idx[emotion] = idx + 1
                        msg = String()
                        msg.data = phrases[idx]
                        self._tts_pub.publish(msg)
                        self.get_logger().info(f'TTS emoción {emotion}: {phrases[idx]}')

        # Algoritmo activo
        algo_msg = String()
        algo_msg.data = str(self._algo)
        self._algo_pub.publish(algo_msg)

    def _switch_algorithm(self, algo_id: int):
        try:
            self._call('/algorithm/switch', {'algorithm': algo_id})
            self._algo = algo_id
            self.get_logger().info(f'Algoritmo cambiado a {algo_id}')
        except Exception as e:
            self.get_logger().error(f'Error cambiando algoritmo: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = HuskyLensWifiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
