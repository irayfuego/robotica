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
  5. NLU hibrido (ver _dispatch): sin wake-word, solo reglas locales (gratis);
     con wake-word ('robot'), la instruccion se manda a Gemini para lenguaje
     natural. Asi solo las frases dirigidas al robot consumen cuota del LLM.

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

try:
    import audioop                       # stdlib (Python <= 3.12); resample/mono
    HAS_AUDIOOP = True
except ImportError:
    HAS_AUDIOOP = False
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
# Se evaluan EN ORDEN y gana la primera que casa: las claves mas especificas
# ('parpadea dos veces', 'mira a la izquierda') van ANTES que las genericas
# ('parpadea', 'mira alrededor'). Claves sin tildes (el texto se normaliza).
DEFAULT_RULES = [
    # ---- dormir / despertar (antes que 'hola': 'buenos dias' despierta) ----
    {'keys': ['despierta', 'despiertate', 'levantate', 'buenos dias'],
     'emotion': 'neutral', 'behavior': 'wake_up', 'say': 'Buenos dias!'},
    {'keys': ['duermete', 'a dormir', 'buenas noches', 'duerme', 've a dormir'],
     'emotion': 'sleeping', 'say': 'Buenas noches.'},

    # ---- emociones ---------------------------------------------------------
    {'keys': ['ponte triste', 'estas triste', 'pon cara triste', 'triste',
              'tristeza', 'pena'],
     'emotion': 'sad'},
    {'keys': ['enfadado', 'enojado', 'enfadate', 'enojate', 'rabia', 'enfado',
              'furioso'],
     'emotion': 'angry'},
    {'keys': ['sorprendido', 'sorpresa', 'asombrado', 'asombro'],
     'emotion': 'surprised'},
    {'keys': ['confundido', 'confuso', 'confusion', 'no entiendo'],
     'emotion': 'confused'},
    {'keys': ['sospecha', 'sospechoso', 'desconfia', 'desconfianza'],
     'emotion': 'suspicious'},
    {'keys': ['cansado', 'sueno', 'agotado', 'fatiga', 'aburrido'],
     'emotion': 'tired'},
    {'keys': ['te quiero', 'enamorado', 'corazon', 'amor', 'carino'],
     'emotion': 'love', 'say': 'Yo tambien te quiero.'},
    {'keys': ['ponte feliz', 'estas feliz', 'alegrate', 'feliz', 'contento',
              'alegre', 'sonrie', 'alegria'],
     'emotion': 'happy', 'say': 'Que alegria!'},
    {'keys': ['neutral', 'tranquilo', 'relajate', 'calma', 'normal'],
     'emotion': 'neutral'},

    # ---- efectos especiales de los ojos ------------------------------------
    {'keys': ['mareado', 'mareo', 'te mareas', 'das vueltas'],
     'behavior': 'dizzy', 'say': 'Uy, que mareo!'},
    {'keys': ['ojos en blanco'],
     'behavior': 'roll_eyes'},
    {'keys': ['dilata las pupilas', 'pupilas grandes'],
     'behavior': 'dilate'},

    # ---- parpadeos y guinos (especificos antes que genericos) --------------
    {'keys': ['parpadea dos veces', 'doble parpadeo'],
     'behavior': 'double_blink'},
    {'keys': ['parpadea despacio', 'parpadeo lento'],
     'behavior': 'slow_blink'},
    {'keys': ['parpadea', 'parpadear', 'parpadeo'],
     'behavior': 'blink'},
    {'keys': ['guina el ojo izquierdo', 'guino izquierdo'],
     'behavior': 'wink_left'},
    {'keys': ['guina', 'guino', 'guiname'],
     'behavior': 'wink_right'},

    # ---- mirada (especificos antes que 'mira alrededor') -------------------
    {'keys': ['mira a la izquierda', 'mira izquierda'],
     'behavior': 'look_left'},
    {'keys': ['mira a la derecha', 'mira derecha'],
     'behavior': 'look_right'},
    {'keys': ['mira arriba', 'mira hacia arriba'],
     'behavior': 'look_up'},
    {'keys': ['mira abajo', 'mira hacia abajo'],
     'behavior': 'look_down'},
    {'keys': ['mira al frente', 'mira al centro', 'mirame'],
     'behavior': 'look_center'},
    {'keys': ['mira alrededor', 'mira a tu alrededor', 'busca', 'explora',
              'echa un vistazo'],
     'behavior': 'look_around'},
    {'keys': ['escanea', 'escaneo', 'rastrea'],
     'behavior': 'scan'},
    {'keys': ['piensa', 'pensando', 'reflexiona'],
     'behavior': 'thinking'},
    {'keys': ['atencion', 'atento', 'alerta'],
     'behavior': 'notice'},

    # ---- social -------------------------------------------------------------
    {'keys': ['como te llamas', 'tu nombre', 'quien eres'],
     'say': 'Soy tu robot, encantado.'},
    {'keys': ['como estas', 'que tal'],
     'say': 'Estoy muy bien, gracias.'},
    {'keys': ['gracias'],
     'emotion': 'happy', 'say': 'De nada!'},
    {'keys': ['adios', 'hasta luego', 'nos vemos', 'chao'],
     'emotion': 'happy', 'say': 'Hasta luego!'},
    {'keys': ['hola', 'buenas tardes', 'saluda', 'saludo'],
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
        'Responde SIEMPRE con JSON valido con exactamente cuatro campos:\n'
        '- emotion: una de [neutral, happy, sad, angry, surprised, confused, '
        'suspicious, tired, love, sleeping] o cadena vacia si no cambia.\n'
        '- intensity: numero entre 0.0 y 1.0, lo intensa que es la emocion '
        '(0.3 leve, 0.6 moderada, 1.0 maxima). Usa 1.0 si dudas.\n'
        '- behavior: una de [blink, double_blink, wink_right, wink_left, '
        'look_around, look_left, look_right, look_up, look_down, look_center, '
        'scan, thinking, dizzy, roll_eyes, notice, dilate, wake_up] '
        'o cadena vacia.\n'
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
                        'emotion':   {'type': 'STRING'},
                        'intensity': {'type': 'NUMBER'},
                        'behavior':  {'type': 'STRING'},
                        'say':       {'type': 'STRING'},
                    },
                    'required': ['emotion', 'intensity', 'behavior', 'say'],
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
            try:
                intensity = max(0.0, min(1.0, float(result.get('intensity', 1.0))))
            except (TypeError, ValueError):
                intensity = 1.0
            return {
                'emotion':   str(result.get('emotion',  '')).strip(),
                'intensity': intensity,
                'behavior':  str(result.get('behavior', '')).strip(),
                'say':       str(result.get('say',      '')).strip(),
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
        # Captura de audio: 'local' = microfono INMP441 por I2S con arecord
        # (sin latencia de red); 'huskylens' = micro de la camara por su MCP.
        self.declare_parameter('audio_source',     'local')
        self.declare_parameter('alsa_device',      'plughw:1,0')
        self.declare_parameter('mic_gain_db',      10.0)   # el INMP441 da nivel bajo
        self.declare_parameter('capture_rate',     48000)  # Hz nativos de la tarjeta I2S
        self.declare_parameter('capture_channels', 2)

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
        self._source     = self.get_parameter('audio_source').value.strip().lower()
        self._alsa_dev   = self.get_parameter('alsa_device').value
        self._mic_gain   = float(self.get_parameter('mic_gain_db').value)
        self._cap_rate   = int(self.get_parameter('capture_rate').value)
        self._cap_ch     = int(self.get_parameter('capture_channels').value)

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

        # Auto-mute mientras el robot habla. Dos fuentes:
        #  - /robot/say: estimacion al publicar (cubre el hueco mientras Piper
        #    genera el audio, antes de que empiece a sonar).
        #  - /robot_eyes/behavior 'speaking'/'speaking_stop': el TTS las publica
        #    al empezar y terminar el audio REAL -> mute exacto, sin depender de
        #    estimaciones (evita que el micro capte el final de la propia voz).
        self.create_subscription(String, '/robot/say', self._cb_say_seen, 10)
        self.create_subscription(String, '/robot_eyes/behavior',
                                 self._cb_behavior_seen, 10)

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
            mic = ('INMP441 local %s' % self._alsa_dev if self._source == 'local'
                   else 'HuskyLens %s' % self._host)
            self.get_logger().info(
                'Comandos de voz ACTIVOS  mic=%s  ventana=%ds  wake=%r  nlu=%s'
                % (mic, self._window, self._wake or '(siempre)',
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

    def _cb_behavior_seen(self, msg):
        b = msg.data.strip()
        if b == 'speaking':
            # El robot empieza a hablar: mute hasta que avise que termina.
            self._mute_until = time.time() + 3600.0
        elif b == 'speaking_stop':
            # Termino el audio: deja un margen anti-eco/reverberacion.
            self._mute_until = time.time() + self._mute_marg

    def _muted(self):
        return time.time() < self._mute_until

    # ------------------------------------------------------------ grabacion
    def _record_loop(self):
        if self._source == 'local':
            self._record_loop_local()
        else:
            self._record_loop_husky()

    def _record_loop_local(self):
        """Captura del INMP441 con reconocimiento en STREAMING: un unico arecord
        continuo alimenta a Vosk chunk a chunk; Vosk detecta el fin de frase
        (endpointing) y devuelve el texto en cuanto dejas de hablar -> minima
        latencia, sin ventanas fijas. Mantener el stream I2S siempre abierto
        ademas evita los 'click' del MAX98357A. Durante el TTS se descarta el
        audio (auto-mute) para no oirse a si mismo."""
        if not HAS_AUDIOOP:
            self.get_logger().error(
                'audioop no disponible (Python >= 3.13?). Captura local imposible; '
                'usa audio_source: huskylens o instala audioop-lts.')
            return
        cmd = ['arecord', '-q', '-D', self._alsa_dev, '-f', 'S32_LE',
               '-r', str(self._cap_rate), '-c', str(self._cap_ch), '-t', 'raw']
        gain = 10.0 ** (self._mic_gain / 20.0)
        while self._running:
            try:
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                        stderr=subprocess.DEVNULL)
            except Exception as e:
                self.get_logger().warn('arecord no arranco: %s' % e)
                time.sleep(1.0)
                continue
            rec = KaldiRecognizer(self._model, self._rate)
            rec.SetWords(False)
            rs_state = None          # estado del resample (streaming)
            partial_on = False       # ya se ha senalado 'listening' esta frase
            try:
                while self._running:
                    chunk = proc.stdout.read(8192)
                    if not chunk:
                        break
                    if self._muted():
                        # Reinicia el reconocedor para no mezclar la voz del
                        # robot con la siguiente frase del usuario.
                        if partial_on or rs_state is not None:
                            rec = KaldiRecognizer(self._model, self._rate)
                            rec.SetWords(False)
                            rs_state = None
                            partial_on = False
                        continue
                    # 48 kHz S32 estereo -> 16 kHz mono 16-bit con ganancia.
                    # L/R del INMP441 a GND -> canal izquierdo.
                    mono = audioop.tomono(chunk, 4, 1, 0)
                    mono = audioop.lin2lin(mono, 4, 2)
                    mono, rs_state = audioop.ratecv(mono, 2, 1, self._cap_rate,
                                                    self._rate, rs_state)
                    if gain != 1.0:
                        mono = audioop.mul(mono, 2, gain)
                    if rec.AcceptWaveform(mono):
                        text = json.loads(rec.Result()).get('text', '').strip()
                        partial_on = False
                        if text:
                            try:
                                self._queue.put_nowait(text)
                            except queue.Full:
                                pass
                    elif not partial_on:
                        p = json.loads(rec.PartialResult()).get('partial', '').strip()
                        if p:
                            # Feedback inmediato: ojos atentos al detectar voz.
                            partial_on = True
                            self._publish(self._pub_behavior, 'listening')
            except Exception as e:
                self.get_logger().warn('captura local fallo: %s' % e)
            finally:
                try:
                    proc.terminate()
                    proc.wait(timeout=2)
                except Exception:
                    pass
            if self._running:
                time.sleep(0.5)   # reintenta el stream si arecord murio

    def _record_loop_husky(self):
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
                item = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                if self._source == 'local':
                    self._handle_text(item)     # item = texto ya transcrito
                else:
                    self._process_husky(item)   # item = slot (descarga + Vosk)
            except Exception as e:
                self.get_logger().warn('Procesado de voz fallo: %s' % e)

    def _handle_text(self, text):
        """Publica y despacha un texto ya reconocido (camino local streaming).
        El dispatch corre aqui, en el worker, para no bloquear la captura."""
        if not text:
            return
        self.get_logger().info('Voz: "%s"' % text)
        m = String(); m.data = text
        self._pub_raw.publish(m)
        self._dispatch(text)

    def _open_ssh_master(self):
        """Abre (o reabre) la conexion SSH maestra reusable hacia la HuskyLens."""
        try:
            subprocess.run(
                ['ssh'] + self._ssh_cm + ['%s@%s' % (self._ssh_user, self._host), 'true'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        except Exception:
            pass

    def _process_husky(self, slot):
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
        self._finish(wav)

    def _finish(self, wav):
        """VAD -> feedback visual -> Vosk -> NLU. Recibe un WAV 16k mono 16-bit
        (comun a la captura local y a la de la HuskyLens)."""
        # VAD por energia: descarta silencio
        if not self._has_speech(wav):
            self._cleanup(wav)
            return

        # Feedback visual inmediato: los ojos "se espabilan" en cuanto se
        # detecta voz, antes de transcribir (la cara de "te he oido").
        self._publish(self._pub_behavior, 'listening')

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
        """NLU hibrido para no malgastar la cuota de Gemini:
          - Si el texto NO menciona el wake-word: SOLO reglas locales (gratis,
            offline). El robot sigue "siempre activo" para comandos basicos
            (ponte feliz, hola...) sin gastar ni una llamada al LLM.
          - Si menciona el wake-word ('robot'): se manda la instruccion (sin la
            palabra) a Gemini para entender lenguaje natural libre. Si Gemini
            falla o no esta configurado, cae a las reglas locales.
        Asi solo las frases dirigidas explicitamente al robot consumen cuota.
        """
        norm = normalize(text)

        # Modo conversacion con LLM: solo cuando se nombra al robot.
        if self._wake and self._wake in norm:
            instruction = norm.split(self._wake, 1)[1].strip() or norm
            if self._gemini:
                # Mirada de "estoy pensando" mientras se espera al LLM; se
                # cancela al llegar la respuesta (o la pisa el TTS al hablar).
                self._publish(self._pub_behavior, 'thinking_loop')
                ok = self._dispatch_gemini(instruction)
                self._publish(self._pub_behavior, 'thinking_loop_stop')
                if ok:
                    return
            norm = instruction   # sin Gemini o fallo -> reglas con la instruccion

        # Reglas locales: unico camino sin wake-word y fallback del LLM.
        self._dispatch_rules(norm)

    def _dispatch_gemini(self, instruction):
        """Llama a Gemini y publica el resultado. True si tuvo exito."""
        result = self._gemini.dispatch(instruction)
        if result is None:
            self.get_logger().warn('Gemini fallo; usando reglas de palabras clave.')
            return False
        if result['emotion']:
            # "emocion:intensidad" -> robot_eyes_node modula la expresion
            intensity = result.get('intensity', 1.0)
            self._publish(self._pub_emotion,
                          '%s:%.2f' % (result['emotion'], intensity))
        if result['behavior']:
            self._publish(self._pub_behavior, result['behavior'])
        if result['say']:
            self._publish(self._pub_say, result['say'])
        self.get_logger().info('Gemini -> %s' % result)
        return True

    def _dispatch_rules(self, norm):
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
