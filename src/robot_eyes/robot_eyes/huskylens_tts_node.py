#!/usr/bin/env python3
"""
huskylens_tts_node.py -- TTS por el altavoz de la HuskyLens.

El Raspberry Pi no tiene altavoz util para el robot; el unico altavoz es el de
la HuskyLens (192.168.1.32). Esta se controla por su propio MCP server, un
servidor JSON-RPC sobre HTTP+SSE que escucha en el puerto 3000 y es accesible
por red desde el Pi (no hace falta ninguna herramienta externa).

Pipeline por cada mensaje en /robot/say:
  1. Piper TTS genera un WAV en el Pi (voz neuronal, mucho mas natural que espeak).
  2. sox lo convierte a MP3 (el reproductor de la HuskyLens solo acepta MP3).
  3. ssh 'cat >' sube el MP3 a /opt/user/mtp/audio/ de la HuskyLens.
  4. tools/call -> multimedia_control(play_music) reproduce el MP3.

Subscriptions:
  /robot/say  (std_msgs/String)  -- texto a pronunciar

Parameters:
  husky_host          IP de la HuskyLens                    (def. 192.168.1.32)
  husky_mcp_port      puerto del MCP server                 (def. 3000)
  husky_ssh_user      usuario SSH de la HuskyLens           (def. root)
  husky_audio_dir     carpeta de audio en la HuskyLens      (def. /opt/user/mtp/audio)
  piper_model         ruta al .onnx de Piper                (def. /home/mimavi/piper-voices/es_ES-davefx-medium.onnx)
  piper_binary        ejecutable de Piper; vacio = python3 -m piper
  piper_length_scale  velocidad: <1.0 mas rapido, >1.0 mas lento  (def. 1.0)
  tts_speed           palabras/min solo para estimar duracion de mute (def. 130)
  volume              volumen de reproduccion 0-100         (def. 90)
  tmp_dir             carpeta temporal en el Pi             (def. /tmp)
  ring_slots          nombres MP3 rotativos                 (def. 4)
"""

import json
import os
import queue
import subprocess
import threading
import time
import urllib.request

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print('[WARN] rclpy not found -- huskylens_tts_node needs ROS to run.')


# --------------------------------------------------------------------------- #
#  Cliente MCP HTTP+SSE minimo (solo stdlib).                                  #
#  El servidor responde a los POST de /message de forma asincrona por el       #
#  stream SSE abierto en /sse; hay que leerlo en un hilo y correlacionar por   #
#  id de JSON-RPC.                                                             #
# --------------------------------------------------------------------------- #
class McpSseClient:
    def __init__(self, base_url, logger=None):
        self._base = base_url.rstrip('/')
        self._log = logger
        self._session_path = None
        self._responses = {}
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self._next_id = 1
        self._id_lock = threading.Lock()
        self._reader_thread = None
        self._sse_resp = None
        self._connected = False

    def _logw(self, msg):
        if self._log:
            self._log(msg)

    def _alloc_id(self):
        with self._id_lock:
            i = self._next_id
            self._next_id += 1
            return i

    def _reader(self):
        try:
            req = urllib.request.Request(self._base + '/sse')
            self._sse_resp = urllib.request.urlopen(req, timeout=30)
            event = None
            for raw in self._sse_resp:
                line = raw.decode('utf-8', 'replace').rstrip('\r\n')
                if line.startswith('event:'):
                    event = line[6:].strip()
                elif line.startswith('data:'):
                    data = line[5:].strip()
                    if event == 'endpoint':
                        self._session_path = data
                        self._ready.set()
                    else:
                        try:
                            msg = json.loads(data)
                        except Exception:
                            continue
                        # El servidor puede emitir escalares (keep-alives, p.ej.
                        # "data: 1"); solo nos interesan las respuestas JSON-RPC.
                        if not isinstance(msg, dict):
                            continue
                        mid = msg.get('id')
                        if mid is not None:
                            with self._lock:
                                self._responses[mid] = msg
                elif line == '':
                    event = None
        except Exception as e:
            self._logw('SSE reader stopped: %s' % e)
        finally:
            self._connected = False
            self._ready.clear()
            self._session_path = None

    def connect(self, timeout=10.0):
        """(Re)abre la sesion SSE y hace el handshake MCP. Idempotente."""
        if self._connected and self._session_path:
            return True
        self._ready.clear()
        self._responses.clear()
        self._reader_thread = threading.Thread(target=self._reader, daemon=True)
        self._reader_thread.start()
        if not self._ready.wait(timeout=timeout):
            self._logw('MCP: no se recibio el endpoint SSE')
            return False
        self._connected = True
        # initialize + notificacion initialized
        init = self._rpc('initialize', {
            'protocolVersion': '2024-11-05',
            'capabilities': {},
            'clientInfo': {'name': 'robot_eyes_tts', 'version': '1.0'},
        }, timeout=timeout)
        if init is None:
            self._logw('MCP: initialize sin respuesta')
            self._connected = False
            return False
        self._notify('notifications/initialized', {})
        return True

    def _post(self, payload):
        data = json.dumps(payload).encode('utf-8')
        req = urllib.request.Request(
            self._base + self._session_path, data=data,
            headers={'Content-Type': 'application/json'}, method='POST')
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status

    def _notify(self, method, params):
        self._post({'jsonrpc': '2.0', 'method': method, 'params': params})

    def _rpc(self, method, params, timeout=15.0):
        mid = self._alloc_id()
        self._post({'jsonrpc': '2.0', 'id': mid, 'method': method,
                    'params': params})
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if mid in self._responses:
                    return self._responses.pop(mid)
            time.sleep(0.05)
        return None

    def call_tool(self, name, arguments, timeout=15.0):
        """Llama a una herramienta MCP, reconectando una vez si la sesion cayo.
        Devuelve (ok, texto_o_error)."""
        for attempt in (1, 2):
            if not (self._connected and self._session_path):
                if not self.connect():
                    continue
            try:
                resp = self._rpc('tools/call',
                                 {'name': name, 'arguments': arguments},
                                 timeout=timeout)
            except Exception as e:
                self._logw('MCP POST fallo (intento %d): %s' % (attempt, e))
                self._connected = False
                continue
            if resp is None:
                self._logw('MCP tools/call sin respuesta (intento %d)' % attempt)
                self._connected = False
                continue
            result = resp.get('result', {})
            text = ''
            for item in result.get('content', []):
                if item.get('type') == 'text':
                    text += item.get('text', '')
            return (not result.get('isError', False), text)
        return (False, 'sin conexion con el MCP server')


# --------------------------------------------------------------------------- #
#  Nodo ROS 2                                                                  #
# --------------------------------------------------------------------------- #
class HuskyLensTtsNode(Node if HAS_ROS else object):

    def __init__(self):
        super().__init__('huskylens_tts_node')

        self.declare_parameter('husky_host',          '192.168.1.32')
        self.declare_parameter('husky_mcp_port',      3000)
        self.declare_parameter('husky_ssh_user',      'root')
        self.declare_parameter('husky_audio_dir',     '/opt/user/mtp/audio')
        self.declare_parameter('piper_model',
                               '/home/mimavi/piper-voices/es_ES-davefx-medium.onnx')
        self.declare_parameter('piper_binary',        '')   # vacio = python3 -m piper
        self.declare_parameter('piper_length_scale',  1.0)
        self.declare_parameter('tts_speed',           130)  # solo para estimar mute
        self.declare_parameter('volume',              90)
        self.declare_parameter('tmp_dir',             '/tmp')
        self.declare_parameter('ring_slots',          4)

        self._host         = self.get_parameter('husky_host').value
        port               = int(self.get_parameter('husky_mcp_port').value)
        self._ssh_user     = self.get_parameter('husky_ssh_user').value
        self._audio_dir    = self.get_parameter('husky_audio_dir').value.rstrip('/')
        self._piper_model  = self.get_parameter('piper_model').value
        piper_bin          = self.get_parameter('piper_binary').value.strip()
        self._piper_cmd    = [piper_bin] if piper_bin else ['python3', '-m', 'piper']
        self._length_scale = str(float(self.get_parameter('piper_length_scale').value))
        self._speed        = int(self.get_parameter('tts_speed').value)
        self._volume       = int(self.get_parameter('volume').value)
        self._tmp          = self.get_parameter('tmp_dir').value.rstrip('/')
        self._slots        = max(1, int(self.get_parameter('ring_slots').value))

        # SSH con conexion maestra persistente (ControlMaster): subir el MP3 con
        # scp tarda ~3.5s en esta WiFi; con conexion reusada baja a ~0.4s.
        self._ssh_cm = [
            '-o', 'BatchMode=yes',
            '-o', 'StrictHostKeyChecking=accept-new',
            '-o', 'ControlMaster=auto',
            '-o', 'ControlPath=/tmp/tts-ssh-%r@%h:%p',
            '-o', 'ControlPersist=120',
        ]

        self._mcp = McpSseClient('http://%s:%d' % (self._host, port),
                                 logger=self.get_logger().warn)

        # Cola de textos. maxsize evita acumulacion si llegan muchos seguidos.
        self._queue = queue.Queue(maxsize=16)
        self._counter = 0
        self._running = True

        self.create_subscription(String, '/robot/say', self._cb_say, 10)

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        # Conectar al MCP en segundo plano para no bloquear el arranque.
        threading.Thread(target=self._mcp.connect, daemon=True).start()

        self.get_logger().info(
            'HuskyLens TTS listo  host=%s  modelo=%s  vol=%d  -- publica en /robot/say'
            % (self._host, os.path.basename(self._piper_model), self._volume))

    # ----------------------------------------------------------- callbacks
    def _cb_say(self, msg):
        text = msg.data.strip()
        if not text:
            return
        try:
            self._queue.put_nowait(text)
        except queue.Full:
            # Descarta el mas antiguo para quedarse con lo mas reciente.
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(text)
            except queue.Empty:
                pass

    # -------------------------------------------------------------- worker
    def _open_ssh_master(self):
        """Abre la conexion SSH maestra reusable hacia la HuskyLens."""
        try:
            subprocess.run(
                ['ssh'] + self._ssh_cm + ['%s@%s' % (self._ssh_user, self._host), 'true'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        except Exception:
            pass

    def _worker_loop(self):
        self._open_ssh_master()
        while self._running:
            try:
                text = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._speak(text)
            except Exception as e:
                self.get_logger().error('TTS fallo: %s' % e)

    def _speak(self, text):
        slot = self._counter % self._slots
        self._counter += 1
        base = 'robot_say_%d.mp3' % slot
        wav = os.path.join(self._tmp, 'robot_say_%d.wav' % slot)
        mp3 = os.path.join(self._tmp, base)

        # 1) Piper TTS -> WAV  (voz neuronal; texto por stdin)
        subprocess.run(
            self._piper_cmd + [
                '--model', self._piper_model,
                '--length_scale', self._length_scale,
                '--output_file', wav,
            ],
            input=text.encode('utf-8'), check=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=30)

        # 2) WAV -> MP3 (el reproductor de la HuskyLens solo acepta MP3)
        subprocess.run(['sox', wav, mp3], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       timeout=30)

        # Duracion real para serializar la voz sin solaparla.
        try:
            out = subprocess.run(['soxi', '-D', mp3], check=True,
                                 capture_output=True, text=True, timeout=10)
            duration = float(out.stdout.strip())
        except Exception:
            duration = max(1.5, len(text.split()) / (self._speed / 60.0))

        try:
            os.remove(wav)
        except OSError:
            pass

        # 3) subir a la HuskyLens con ssh 'cat >' sobre la conexion maestra
        #    (un solo round-trip; mucho mas rapido que scp en esta WiFi)
        remote = '%s/%s' % (self._audio_dir, base)
        with open(mp3, 'rb') as fh:
            subprocess.run(
                ['ssh'] + self._ssh_cm + ['%s@%s' % (self._ssh_user, self._host),
                                          "cat > '%s'" % remote],
                stdin=fh, check=True, stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, timeout=30)

        # 4) reproducir via MCP
        self.get_logger().info('TTS say (husky): "%s"' % text)
        ok, info = self._mcp.call_tool(
            'multimedia_control',
            {'operation': 'play_music', 'filename': base, 'volume': self._volume})
        if not ok:
            self.get_logger().warn('play_music fallo: %s' % info)
            return

        # Esperar a que termine la reproduccion antes del siguiente texto.
        time.sleep(duration + 0.4)

    def shutdown(self):
        self._running = False


def main(args=None):
    if not HAS_ROS:
        return
    rclpy.init(args=args)
    node = HuskyLensTtsNode()
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
