#!/usr/bin/env python3
"""
tts_node.py — Text-to-Speech para el robot Mecanum
Suscripción: /tts/say  (std_msgs/String)  → habla el texto
             /tts/stop (std_msgs/Empty)    → para la reproducción
Parámetros:
  piper_model  (str)  ruta al modelo .onnx
  piper_bin    (str)  ruta al binario piper
  audio_device (str)  dispositivo ALSA (default: 'default')
  volume       (int)  volumen 0-100
"""

import subprocess
import threading
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty


class TTSNode(Node):

    def __init__(self):
        super().__init__('tts_node')

        self.declare_parameter('piper_model',
            '/home/mimavi/piper_models/es_ES-sharvard-medium.onnx')
        self.declare_parameter('piper_bin',
            '/home/mimavi/.local/bin/piper')
        self.declare_parameter('audio_device', 'default')
        self.declare_parameter('volume', 90)

        self._model  = self.get_parameter('piper_model').value
        self._piper  = self.get_parameter('piper_bin').value
        self._device = self.get_parameter('audio_device').value
        self._volume = self.get_parameter('volume').value

        self._lock    = threading.Lock()
        self._current = None   # proceso aplay en curso

        self.create_subscription(String, '/tts/say',  self._on_say,  10)
        self.create_subscription(Empty,  '/tts/stop', self._on_stop, 10)

        self.get_logger().info(
            f'tts_node listo — modelo: {self._model}')

    def _on_say(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f'TTS: "{text}"')
        threading.Thread(target=self._speak, args=(text,), daemon=True).start()

    def _on_stop(self, _msg):
        self._kill_current()

    def _kill_current(self):
        with self._lock:
            if self._current and self._current.poll() is None:
                self._current.terminate()
                self._current = None

    def _speak(self, text: str):
        self._kill_current()
        wav = '/tmp/tts_output.wav'
        try:
            # Generar WAV con piper
            subprocess.run(
                [self._piper,
                 '--model', self._model,
                 '--output_file', wav],
                input=text.encode(),
                check=True,
                capture_output=True,
            )
            # Reproducir con aplay
            with self._lock:
                self._current = subprocess.Popen(
                    ['aplay', '-D', self._device, wav],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            self._current.wait()
        except Exception as e:
            self.get_logger().error(f'Error TTS: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
