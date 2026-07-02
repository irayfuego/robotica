#!/usr/bin/env python3
"""Panel web de control del robot (puerto 8080).

Permite, sin SSH ni asistente: resetear el mapa SLAM, cambiar el modo del
robot (mapeo / navegacion / completo), guardar el mapa y ver estado y log.
Solo stdlib (http.server): sin dependencias. Pensado para la LAN de casa.
Servicio systemd: robot-panel.service.
"""
import json
import os
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MODE_FILE = "/home/mimavi/robot_mode"
LOG_FILE = "/tmp/robot_bringup.log"
MAPS_DIR = "/home/mimavi/maps"
MODES = {"mapping": "Mapeo (SLAM + mando)",
         "navigation": "Navegación (Nav2 + mapa guardado)",
         "full": "Completo (todo, carga alta)"}

_lock = threading.Lock()  # una accion pesada a la vez


def sh(cmd, timeout=60):
    """Ejecuta un comando de shell y devuelve (exit, salida)."""
    try:
        r = subprocess.run(["bash", "-c", cmd], capture_output=True,
                           text=True, timeout=timeout)
        out = (r.stdout + r.stderr).strip()
        return r.returncode, out
    except subprocess.TimeoutExpired:
        return 124, "timeout tras %ss" % timeout


def get_mode():
    try:
        m = open(MODE_FILE).read().strip()
        return m if m in MODES else "mapping"
    except OSError:
        return "mapping"


def status():
    _, active = sh("systemctl is-active robot-bringup.service", 10)
    _, load = sh("cut -d' ' -f1-3 /proc/loadavg", 5)
    _, nodes = sh("ps -eo comm | grep -cE 'joy_linux|m1ct_d2|rrc_lite|async_slam|ekf_node|teleop|foxglove|controller_serv'", 5)
    _, joy = sh("test -e /dev/input/js0 && echo si || echo NO", 5)
    _, throttled = sh("vcgencmd get_throttled 2>/dev/null | cut -d= -f2", 5)
    _, uptime = sh("uptime -p", 5)
    return {"servicio": active, "modo": get_mode(), "carga": load,
            "nodos_ros": nodes, "mando_js0": joy,
            "throttled": throttled or "?", "uptime": uptime}


def do_action(action, arg):
    if action == "status":
        return 0, json.dumps(status(), ensure_ascii=False, indent=1)
    if action == "log":
        return sh("tail -30 %s" % LOG_FILE, 10)
    if action == "reset_map":
        return sh("sudo -n systemctl restart robot-bringup.service && echo 'Reiniciado: mapa en blanco. Espera ~25s y reconecta Foxglove.'", 90)
    if action == "set_mode":
        if arg not in MODES:
            return 1, "modo invalido: %r" % arg
        open(MODE_FILE, "w").write(arg + "\n")
        code, out = sh("sudo -n systemctl restart robot-bringup.service", 90)
        return code, ("Modo '%s' aplicado y servicio reiniciado. Espera ~25s "
                      "y reconecta Foxglove.\n%s" % (MODES[arg], out))
    if action == "save_map":
        name = "".join(c for c in (arg or "casa") if c.isalnum() or c in "-_") or "casa"
        cmd = ("mkdir -p %s && source /opt/ros/jazzy/setup.bash && "
               "source /home/mimavi/robotica_ws/install/setup.bash && "
               "timeout 40 ros2 run nav2_map_server map_saver_cli -f %s/%s 2>&1 | tail -4 && "
               "ls -la %s/%s.*" % (MAPS_DIR, MAPS_DIR, name, MAPS_DIR, name))
        return sh(cmd, 60)
    if action == "stop_robot":
        return sh("sudo -n systemctl stop robot-bringup.service && echo 'Robot PARADO (servicio detenido).'", 60)
    return 1, "accion desconocida: %r" % action


PAGE = """<!DOCTYPE html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot — Panel de control</title>
<style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:16px;max-width:640px;margin:auto}
 h1{font-size:1.3em}
 button{display:block;width:100%;margin:8px 0;padding:14px;font-size:1.05em;border:0;border-radius:10px;cursor:pointer;font-weight:600}
 .b-map{background:#2d6cdf;color:#fff}.b-nav{background:#7a3fd1;color:#fff}
 .b-reset{background:#e67e22;color:#fff}.b-save{background:#27ae60;color:#fff}
 .b-stop{background:#c0392b;color:#fff}.b-info{background:#333;color:#eee}
 pre{background:#000;border:1px solid #333;border-radius:8px;padding:10px;white-space:pre-wrap;font-size:.85em;min-height:90px}
 input{width:100%;padding:10px;border-radius:8px;border:1px solid #444;background:#222;color:#eee;box-sizing:border-box}
 .busy{opacity:.5;pointer-events:none}
 small{color:#888}
</style></head><body>
<h1>🤖 Panel del robot</h1>
<div id="wrap">
<button class="b-reset" onclick="act('reset_map')">🔄 Resetear mapa (reinicia modo actual)</button>
<button class="b-map" onclick="act('set_mode','mapping')">🗺️ Modo MAPEO (SLAM + mando)</button>
<button class="b-nav" onclick="act('set_mode','navigation')">🧭 Modo NAVEGACIÓN (Nav2 + mapa)</button>
<input id="mapname" placeholder="nombre del mapa (por defecto: casa)">
<button class="b-save" onclick="act('save_map',document.getElementById('mapname').value)">💾 Guardar mapa</button>
<button class="b-info" onclick="act('status')">📊 Estado</button>
<button class="b-info" onclick="act('log')">📜 Últimas líneas del log</button>
<button class="b-stop" onclick="if(confirm('¿Parar TODO el robot?'))act('stop_robot')">⛔ Parar robot</button>
</div>
<pre id="out">Pulsa un botón…</pre>
<small>Foxglove: ws://192.168.1.233:8765 — tras reset/cambio de modo, reconecta.</small>
<script>
async function act(a, arg){
 const w=document.getElementById('wrap'), o=document.getElementById('out');
 w.classList.add('busy'); o.textContent='⏳ '+a+' …';
 try{
  const r=await fetch('/action',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({action:a,arg:arg||''})});
  const j=await r.json();
  o.textContent=(j.ok?'✅ ':'❌ ')+j.out;
 }catch(e){o.textContent='❌ error de red: '+e;}
 w.classList.remove('busy');
}
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass  # sin ruido en el journal

    def _send(self, code, body, ctype):
        data = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype + "; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, PAGE, "text/html")
        else:
            self._send(404, "no existe", "text/plain")

    def do_POST(self):
        if self.path != "/action":
            self._send(404, "no existe", "text/plain")
            return
        try:
            n = int(self.headers.get("Content-Length", 0))
            req = json.loads(self.rfile.read(n) or b"{}")
            with _lock:
                code, out = do_action(req.get("action", ""), req.get("arg", ""))
            self._send(200, json.dumps({"ok": code == 0, "out": out},
                                       ensure_ascii=False), "application/json")
        except Exception as e:  # noqa: BLE001 — el panel nunca debe caerse
            self._send(200, json.dumps({"ok": False, "out": "error: %s" % e},
                                       ensure_ascii=False), "application/json")


if __name__ == "__main__":
    ThreadingHTTPServer(("0.0.0.0", 8080), Handler).serve_forever()
