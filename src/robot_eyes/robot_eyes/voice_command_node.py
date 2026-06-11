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
  5. NLU: Gemini 2.0 Flash si hay clave API; si no, palabras clave locales.

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
  gemini_api_key  clave API de Gemini; si vacia, usa variable GEMINI_API_KEY.
                  Preferir variable de entorno: no ponerla en el YAML (es publico).
"""

import json
import os
import queue
import subprocess
import threading
import time
import urllib.error
import urllib.request
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
#  NLU por palabras clave (fallback cuando Gemini no esta configurado o falla) #
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


# --------------------------------------------------------------------------- #
#  Cliente Gemini 2.0 Flash (solo stdlib).                                     #
#  Interpreta la transcripcion de voz y devuelve JSON con emotion/behavior/say #
# --------------------------------------------------------------------------- #
class GeminiClient:

    _URL = ('https://generativelanguage.googleapis.com/v1beta/models/'
            '%s:generateContent?key=%s')

    _SYSTEM = (
        'Eres el cerebro de un robot amigable con ojos animados. '
        'Recibes lo que el usuario acaba de decir por voz (transcripcion Vosk, '
        'puede tener errores menores) y decides como reacciona el robot. '
        'Responde SIEMPRE con JSON valido con exactamente tres campos:\n'
        '- emotion: una de [neutral, happy, sad, angry, surprised, confused, '
        'suspicious, tired, love, sleeping] o cadena vacia si no cambia.\n'
        '- behavior: una de [blink, look_around] o cadena vacia.\n'
        '- say: frase corta en espanol que dira el robot en voz alta, '
        'o cadena vacia si no tiene nada que decir.\n'
        'El robot es simpatico, curioso y expresivo. Responde de forma natural '
        'y breve. Habla siempre en espanol.'
    )

    def __init__(self, api_key, model='gemini-2.5-flash-lite', logger=None):
        self._key = api_key
        self._model = model
        self._log = logger

    def dispatch(self, text, timeout=8.0):
        """Llama a Gemini y devuelve dict {emotion, behavior, say}, o None si falla."""
        payload = {
            'systemInstruction': {'parts': [{'text': self._SYSTEM}]},
            'contents': [{'role': 'user', 'parts': [{'text': text}]}],
            'generationConfig': {
                'temperature': 0,
                'responseMimeType': 'application/json',
                'responseSchema': {
                    'type': 'OBJECT',
                    'properties': {
                        'emotion':  {'type': 'STRING'},
                        'behavior': {'type': 'STRING'},
                        'say':      {'type': 'STRING'},
                    },
                    'required': ['emotion', 'behavior', 'say'],
                },
            },
        }
        data = json.dumps(payload).encode('utf-8')
        req = urllib.request.Request(
            self._URL % (self._model, self._key), data=data,
            headers={'Content-Type': 'application/json'}, method='POST')
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                body = json.loads(r.read().decode('utf-8'))
        except urllib.error.HTTPError as e:
            if self._log:
                self._log('Gemini HTTP %d: %s'
                          % (e.code, e.read().decode('utf-8', 'replace')[:200]))
            return None
        except Exception as e:
            if self._log:
                self._log('Gemini error: %s' % e)
            return None
        try:
            raw = body['candidates'][0]['content']['parts'][0]['text']
            result = json.loads(raw)
            return {
                'emotion':  str(result.get('emotion',  '')).strip(),
                'behavior': str(result.get('behavior', '')).strip(),
                'say':      str(result.get('say',      '')).strip(),
            }
        except Exception as e:
            if self._log:
                self._log('Gemini parse error: %s  body=%s' % (e, str(body)[:300]))
            return None


# --------------------------------------------------------------------------- #
#  Nodo ROS 2                                                                  #
# --------------------------------------------------------------------------- #
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
        # Clave Gemini: si vacia, lee GEMINI_API_KEY del entorno.
        # NO poner el valor real en el YAML (el repo es publico).
        self.declare_parameter('gemini_api_key',   '')
        self.declare_parameter('gemini_model',     'gemini-2.5-flash-lite')

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

        # Opciones SSH con conexion maestra persistente (ControlMaster): evita
        # el handshake en cada transferencia. La WiFi de la HuskyLens es lenta
        # (scp con handshake ~2s; reusando conexion ~0.5s).
        self._ssh_cm = [
            '-o', 'BatchMode=yes',
            '-o', 'StrictHostKeyChecking=accept-new',
            '-o', 'ControlMaster=auto',
            '-o', 'ControlPath=/tmp/vc-ssh-%r@%h:%p',
            '-o', 'ControlPersist=120',
        ]

        # NLU: Gemini si hay clave; si no, palabras clave locales como fallback.
        api_key = (self.get_parameter('gemini_api_key').value
                   or os.environ.get('GEMINI_API_KEY', ''))
        gemini_model = self.get_parameter('gemini_model').value
        if api_key:
            self._gemini = GeminiClient(api_key, model=gemini_model,
                                        logger=self.get_logger().warn)
            self.get_logger().info('NLU: Gemini activo  modelo=%s.' % gemini_model)
        else:
            self._gemini = None
            self.get_logger().info(
                'NLU: palabras clave locales (configura GEMINI_API_KEY para LLM).')

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
                'Comandos de voz ACTIVOS  host=%s  ventana=%ds  wake=%r  nlu=%s'
                % (self._host, self._window, self._wake or '(siempre)',
                   'gemini' if self._gemini else 'reglas'))
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
        self._open_ssh_master()
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

    def _open_ssh_master(self):
        """Abre (o reabre) la conexion SSH maestra reusable hacia la HuskyLens."""
        try:
            subprocess.run(
                ['ssh'] + self._ssh_cm + ['%s@%s' % (self._ssh_user, self._host), 'true'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        except Exception:
            pass

    def _process(self, slot):
        base = 'voice_rec_%d.mp3' % slot
        mp3 = os.path.join(self._tmp, base)
        wav = os.path.join(self._tmp, 'voice_rec_%d.wav' % slot)

        # Traer el MP3 con 'ssh cat' sobre la conexion maestra (un solo
        # round-trip de datos, mucho mas rapido que scp en esta WiFi). Con
        # reintentos por si el fichero tarda unas decimas en finalizar.
        remote = "%s/%s" % (self._audio_dir, base)
        ok = False
        for attempt in range(3):
            with open(mp3, 'wb') as fh:
                r = subprocess.run(
                    ['ssh'] + self._ssh_cm + ['%s@%s' % (self._ssh_user, self._host),
                                              "cat '%s'" % remote],
                    stdout=fh, stderr=subprocess.DEVNULL, timeout=15)
            if r.returncode == 0 and os.path.getsize(mp3) > 0:
                ok = True
                break
            time.sleep(0.5)
        if not ok:
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

        # Intentar primero con Gemini LLM
        if self._gemini:
            result = self._gemini.dispatch(text)
            if result is not None:
                if result['emotion']:
                    self._publish(self._pub_emotion, result['emotion'])
                if result['behavior']:
                    self._publish(self._pub_behavior, result['behavior'])
                if result['say']:
                    self._publish(self._pub_say, result['say'])
                self.get_logger().info('Gemini -> %s' % result)
                return
            self.get_logger().warn('Gemini fallo; usando reglas de palabras clave.')

        # Fallback: palabras clave locales
        for rule in self._rules:
            if any(k in norm for k in rule['keys']):
                if 'emotion' in rule:
                    self._publish(self._pub_emotion, rule['emotion'])
                if 'behavior' in rule:
                    self._publish(self._pub_behavior, rule['behavior'])
                if 'say' in rule:
                    self._publish(self._pub_say, rule['say'])
                self.get_logger().info(
                    'Comando reconocido (reglas) -> %s' % {k: v for k, v in rule.items()
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
