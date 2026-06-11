#!/usr/bin/env python3
"""
voice_command_node.py -- comandos de voz en lenguaje natural, siempre activo.

El microfono esta en la HuskyLens 2 y su PCM de captura lo tiene tomado el
firmware (/opt/menu), asi que NO se puede usar arecord directo. La captura se
hace por el MCP server de la HuskyLens (start_recording_audio), igual que la
reproduccion del TTS. El reconocimiento es offline con Vosk (sin internet, sin
enviar audio a la nube).

Flujo (escucha continua):
  1. Bucle: start_recording_audio(duration=W) en la HuskyLens, alternando dos
     slots para que la escucha sea casi continua.
  2. Por cada ventana grabada: scp del MP3 al Pi -> sox a WAV 16k mono.
  3. VAD por energia RMS: descarta ventanas en silencio (ahorra CPU de STT).
  4. Vosk transcribe el WAV a texto.
  5. NLU por palabras clave -> publica emocion / comportamiento / habla.

Para que el robot no se oiga a si mismo, se silencia (no graba ni procesa)
mientras suena el TTS: escucha /robot/say y se auto-mutea un tiempo estimado.

Subscriptions:
  /robot/say  (std_msgs/String)  -- para auto-mute mientras el robot habla

Publications:
  /robot/voice_raw     (std_msgs/String)  -- transcripcion cruda de Vosk
  /robot_eyes/emotion  (std_msgs/String)  -- emocion derivada del comando
  /robot_eyes/behavior (std_msgs/String)  -- comportamiento derivado del comando
  /robot/say           (std_msgs/String)  -- respuesta hablada del robot

Parameters (ver robot_eyes_params.yaml, seccion voice_command_node).
"""

import json
import os
import queue
import subprocess
import threading
import time
import wave

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print('[WARN] rclpy not found -- voice_command_node needs ROS to run.')

try:
    from vosk import Model, KaldiRecognizer
    HAS_VOSK = True
except ImportError:
    HAS_VOSK = False

# Reutiliza el cliente MCP HTTP+SSE del nodo de TTS (mismo servidor, puerto 3000)
from .huskylens_tts_node import McpSseClient


# --------------------------------------------------------------------------- #
#  NLU minimo por palabras clave (espanol).                                    #
#  Cada regla: 'keys' (cualquiera presente dispara) + acciones opcionales      #
#  'emotion' / 'behavior' / 'say'. Se evaluan en orden; la primera que case    #
#  ejecuta sus acciones y se detiene.                                          #
#  Las claves van SIN tildes; el texto reconocido tambien se normaliza sin     #
#  tildes para casar de forma robusta (Vosk small a veces no acentua).         #
# --------------------------------------------------------------------------- #
DEFAULT_RULES = [
    {'keys': ['ponte triste', 'estas triste', 'pon cara triste', 'triste', 'tristeza'],
     'emotion': 'sad'},
    {'keys': ['enfadado', 'enojado', 'enfadate', 'enojate', 'rabia', 'enfado'],
     'emotion': 'angry'},
    {'keys': ['sorprendido', 'sorpresa', 'asombrado', 'asombro'],
     'emotion': 'surprised'},
    {'keys': ['confundido', 'confuso', 'confusion', 'no entiendo'],
     'emotion': 'confused'},
    {'keys': ['sospecha', 'sospechoso', 'desconfia', 'desconfianza'],
     'emotion': 'suspicious'},
    {'keys': ['cansado', 'sueno', 'agotado', 'fatiga'],
     'emotion': 'tired'},
    {'keys': ['te quiero', 'enamorado', 'corazon', 'amor', 'carino'],
     'emotion': 'love', 'say': 'Yo tambien te quiero.'},
    {'keys': ['duermete', 'a dormir', 'buenas noches', 'duerme', 've a dormir'],
     'emotion': 'sleeping', 'say': 'Buenas noches.'},
    {'keys': ['ponte feliz', 'estas feliz', 'alegrate', 'feliz', 'contento',
              'alegre', 'sonrie', 'alegria'],
     'emotion': 'happy', 'say': 'Que alegria!'},
    {'keys': ['neutral', 'tranquilo', 'relajate', 'calma', 'normal'],
     'emotion': 'neutral'},
    {'keys': ['parpadea', 'parpadear', 'guina'],
     'behavior': 'blink'},
    {'keys': ['mira alrededor', 'mira a tu alrededor', 'busca', 'explora',
              'echa un vistazo'],
     'behavior': 'look_around'},
    {'keys': ['como te llamas', 'tu nombre', 'quien eres'],
     'say': 'Soy tu robot, encantado.'},
    {'keys': ['como estas', 'que tal'],
     'say': 'Estoy muy bien, gracias.'},
    {'keys': ['adios', 'hasta luego', 'nos vemos', 'chao'],
     'emotion': 'happy', 'say': 'Hasta luego!'},
    {'keys': ['hola', 'buenos dias', 'buenas tardes', 'saluda', 'saludo'],
     'emotion': 'happy', 'say': 'Hola! Como estas?'},
]

def normalize(text):
    """Minusculas y sin tildes/dieresis, para casar palabras clave de forma robusta
    (Vosk small a veces no acentua)."""
    text = text.lower()
    pairs = {
        'á': 'a', 'é': 'e', 'í': 'i', 'ó': 'o', 'ú': 'u',
        'ü': 'u', 'ñ': 'n',
    }
    for k, v in pairs.items():
        text = text.replace(k, v)
    return text


class VoiceCommandNode(Node if HAS_ROS else object):

    def __init__(self):
        super().__init__('voice_command_node')

        self.declare_parameter('husky_host',       '192.168.1.32')
        self.declare_parameter('husky_mcp_port',   3000)
        self.declare_parameter('husky_ssh_user',   'root')
        self.declare_parameter('husky_audio_dir',  '/opt/user/mtp/audio')
        self.declare_parameter('model_path',       '/home/mimavi/vosk-model-es')
        self.declare_parameter('window_sec',       5)
        self.declare_parameter('rec_margin',       3.5)
        self.declare_parameter('sample_rate',      16000)
        self.declare_parameter('vad_rms_threshold', 0.010)
        self.declare_parameter('wake_word',        '')      # vacio = siempre activo
        self.declare_parameter('self_mute_margin', 1.5)
        self.declare_parameter('tts_speed',        130)     # para estimar duracion del mute
        self.declare_parameter('tmp_dir',          '/tmp')
        self.declare_parameter('enabled',          True)

        self._host       = self.get_parameter('husky_host').value
        port             = int(self.get_parameter('husky_mcp_port').value)
        self._ssh_user   = self.get_parameter('husky_ssh_user').value
        self._audio_dir  = self.get_parameter('husky_audio_dir').value.rstrip('/')
        self._model_path = self.get_parameter('model_path').value
        self._window     = max(2, int(self.get_parameter('window_sec').value))
        self._margin     = max(2.5, float(self.get_parameter('rec_margin').value))
        self._rate       = int(self.get_parameter('sample_rate').value)
        self._vad_thr    = float(self.get_parameter('vad_rms_threshold').value)
        self._wake       = normalize(self.get_parameter('wake_word').value.strip())
        self._mute_marg  = float(self.get_parameter('self_mute_margin').value)
        self._tts_speed  = int(self.get_parameter('tts_speed').value)
        self._tmp        = self.get_parameter('tmp_dir').value.rstrip('/')
        self._enabled    = bool(self.get_parameter('enabled').value)

        self._rules = DEFAULT_RULES
        self._mute_until = 0.0
        self._running = True

        # Publicadores
        self._pub_raw      = self.create_publisher(String, '/robot/voice_raw',     10)
        self._pub_emotion  = self.create_publisher(String, '/robot_eyes/emotion',  10)
        self._pub_behavior = self.create_publisher(String, '/robot_eyes/behavior', 10)
        self._pub_say      = self.create_publisher(String, '/robot/say',           10)

        # Auto-mute mientras el robot habla
        self.create_subscription(String, '/robot/say', self._cb_say_seen, 10)

        if not HAS_VOSK:
            self.get_logger().error(
                'Vosk no esta instalado (pip install vosk). Nodo inactivo.')
            return
        if not os.path.isdir(self._model_path):
            self.get_logger().error(
                'Modelo Vosk no encontrado en %s. Nodo inactivo.' % self._model_path)
            return

        self.get_logger().info('Cargando modelo Vosk desde %s ...' % self._model_path)
        self._model = Model(self._model_path)
        self.get_logger().info('Modelo Vosk cargado.')

        self._mcp = McpSseClient('http://%s:%d' % (self._host, port),
                                 logger=self.get_logger().warn)
        self._queue = queue.Queue(maxsize=8)

        if self._enabled:
            threading.Thread(target=self._record_loop, daemon=True).start()
            threading.Thread(target=self._worker_loop, daemon=True).start()
            self.get_logger().info(
                'Comandos de voz ACTIVOS  host=%s  ventana=%ds  wake=%r'
                % (self._host, self._window, self._wake or '(siempre)'))
        else:
            self.get_logger().info('Comandos de voz deshabilitados (enabled=false).')

    # ----------------------------------------------------------- auto-mute
    def _cb_say_seen(self, msg):
        text = msg.data.strip()
        if not text:
            return
        words = max(1, len(text.split()))
        dur = words / (self._tts_speed / 60.0)
        self._mute_until = time.time() + dur + self._mute_marg

    def _muted(self):
        return time.time() < self._mute_until

    # ------------------------------------------------------------ grabacion
    def _record_loop(self):
        if not self._mcp.connect():
            self.get_logger().warn('No se pudo conectar al MCP para grabar; reintentando...')
        slot = 0
        while self._running:
            # Si el robot esta hablando, espera a que termine antes de grabar
            # (el micro captaria la propia voz del robot).
            while self._muted() and self._running:
                time.sleep(0.2)
            if not self._running:
                break
            self._start_recording(slot)
            # El firmware tarda ~3s EXTRA en finalizar el MP3 tras la ventana;
            # lanzar otra grabacion antes ABORTA la anterior (medido: 5s de
            # audio -> fichero visible a los ~8s). De ahi el margen.
            time.sleep(self._window + self._margin)
            # Si la ventana cayo en periodo de mute (TTS), se descarta.
            if not self._muted():
                try:
                    self._queue.put_nowait(slot)
                except queue.Full:
                    pass
            slot = 1 - slot

    def _start_recording(self, slot):
        ok, info = self._mcp.call_tool(
            'multimedia_control',
            {'operation': 'start_recording_audio', 'duration': self._window,
             'filename': 'voice_rec_%d.mp3' % slot})
        if not ok:
            self.get_logger().warn('start_recording_audio fallo: %s' % info)

    # ----------------------------------------------------------- procesado
    def _worker_loop(self):
        while self._running:
            try:
                slot = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._process(slot)
            except Exception as e:
                self.get_logger().warn('Procesado de voz fallo: %s' % e)

    def _process(self, slot):
        base = 'voice_rec_%d.mp3' % slot
        mp3 = os.path.join(self._tmp, base)
        wav = os.path.join(self._tmp, 'voice_rec_%d.wav' % slot)

        # Traer el MP3 grabado desde la HuskyLens (con reintentos: el fichero
        # puede tardar unas decimas mas en aparecer finalizado)
        src = '%s@%s:%s/%s' % (self._ssh_user, self._host, self._audio_dir, base)
        for attempt in range(3):
            r = subprocess.run(
                ['scp', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=accept-new',
                 src, mp3],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=20)
            if r.returncode == 0:
                break
            time.sleep(0.7)
        else:
            self.get_logger().warn('No se pudo traer %s tras 3 intentos' % base)
            return

        # MP3 -> WAV 16k mono 16-bit (formato que Vosk necesita)
        subprocess.run(
            ['sox', mp3, '-r', str(self._rate), '-c', '1', '-b', '16', wav],
            check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=20)

        # VAD por energia: descarta silencio
        if not self._has_speech(wav):
            self._cleanup(wav)
            return

        text = self._transcribe(wav)
        self._cleanup(wav)
        if not text:
            return

        self.get_logger().info('Voz: "%s"' % text)
        m = String(); m.data = text
        self._pub_raw.publish(m)
        self._dispatch(text)

    def _has_speech(self, wav_path):
        try:
            out = subprocess.run(['sox', wav_path, '-n', 'stat'],
                                 capture_output=True, text=True, timeout=10)
            for line in out.stderr.splitlines():
                if 'RMS' in line and 'amplitude' in line:
                    rms = float(line.split(':')[1].strip())
                    return rms >= self._vad_thr
        except Exception:
            return True   # ante la duda, intenta transcribir
        return True

    def _transcribe(self, wav_path):
        wf = wave.open(wav_path, 'rb')
        rec = KaldiRecognizer(self._model, wf.getframerate())
        rec.SetWords(False)
        text = ''
        while True:
            data = wf.readframes(4000)
            if len(data) == 0:
                break
            if rec.AcceptWaveform(data):
                text += ' ' + json.loads(rec.Result()).get('text', '')
        text += ' ' + json.loads(rec.FinalResult()).get('text', '')
        wf.close()
        return text.strip()

    # ------------------------------------------------------------------ NLU
    def _dispatch(self, text):
        norm = normalize(text)

        if self._wake:
            if self._wake not in norm:
                return
            norm = norm.split(self._wake, 1)[1].strip()

        for rule in self._rules:
            if any(k in norm for k in rule['keys']):
                if 'emotion' in rule:
                    self._publish(self._pub_emotion, rule['emotion'])
                if 'behavior' in rule:
                    self._publish(self._pub_behavior, rule['behavior'])
                if 'say' in rule:
                    self._publish(self._pub_say, rule['say'])
                self.get_logger().info(
                    'Comando reconocido -> %s' % {k: v for k, v in rule.items()
                                                  if k != 'keys'})
                return

    def _publish(self, pub, value):
        m = String(); m.data = value
        pub.publish(m)

    # -------------------------------------------------------------- helpers
    def _cleanup(self, *paths):
        for p in paths:
            try:
                os.remove(p)
            except OSError:
                pass

    def shutdown(self):
        self._running = False


def main(args=None):
    if not HAS_ROS:
        return
    rclpy.init(args=args)
    node = VoiceCommandNode()
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
