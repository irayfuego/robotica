#!/usr/bin/env python3
"""Panel web de control del robot (puerto 8080) — v2.

Ademas de las acciones basicas (reset de mapa, cambio de modo, guardar mapa,
estado, log, parar), incorpora: ajustes de velocidad del mando, presets de
SLAM, gestion de mapas (elegir el activo para navegacion, ver, borrar),
mantenimiento (temperatura/tension, apagado seguro, reinicio, prueba de
motores, test del mando) y reinicio de los ojos.
Solo stdlib + PyYAML (presente por ROS). Servicio: robot-panel.service.
"""
import json
import os
import re
import struct
import subprocess
import threading
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

WS = "/home/mimavi/robotica_ws"
TELEOP_YAML = WS + "/src/robotica_bringup/config/8bitdo_teleop.yaml"
SLAM_YAML = WS + "/src/robotica_bringup/config/slam_toolbox.yaml"
MODE_FILE = "/home/mimavi/robot_mode"
MAP_FILE = "/home/mimavi/robot_map"      # yaml del mapa activo para navegacion
LOG_FILE = "/tmp/robot_bringup.log"
MAPS_DIR = "/home/mimavi/maps"
MODES = {"mapping": "Mapeo", "navigation": "Navegación", "full": "Completo"}

_lock = threading.Lock()


def sh(cmd, timeout=60):
    try:
        r = subprocess.run(["bash", "-c", cmd], capture_output=True,
                           text=True, timeout=timeout)
        return r.returncode, (r.stdout + r.stderr).strip()
    except subprocess.TimeoutExpired:
        return 124, "timeout tras %ss" % timeout


def rebuild_and_restart():
    """Recompila robotica_bringup (rapido, ~15s) y reinicia el bringup."""
    code, out = sh("cd %s && source /opt/ros/jazzy/setup.bash && "
                   "colcon build --symlink-install --packages-select robotica_bringup "
                   "2>&1 | tail -1 && sudo -n systemctl restart robot-bringup.service"
                   % WS, 150)
    return code, out + "\nServicio reiniciado: espera ~25s y reconecta Foxglove."


def get_mode():
    try:
        m = open(MODE_FILE).read().strip()
        return m if m in MODES else "mapping"
    except OSError:
        return "mapping"


def get_active_map():
    try:
        return open(MAP_FILE).read().strip()
    except OSError:
        return MAPS_DIR + "/casa.yaml"


def get_speeds():
    s = open(TELEOP_YAML, encoding="utf-8").read()
    def grab(block, key):
        m = re.search(block + r":\s*\n\s*" + key + r":\s*(-?[\d.]+)", s)
        return abs(float(m.group(1))) if m else None
    return {"v": grab("scale_linear", "x"), "vt": grab("scale_linear_turbo", "x"),
            "w": grab("scale_angular", "yaw"), "wt": grab("scale_angular_turbo", "yaw")}


def set_speeds(v, vt, w, wt):
    v, vt, w, wt = (round(float(x), 2) for x in (v, vt, w, wt))
    if not (0.05 <= v <= 1.0 and 0.05 <= vt <= 1.2):
        return 1, "velocidad lineal fuera de rango (0.05-1.0 / turbo 1.2)"
    if not (0.2 <= w <= 3.0 and 0.2 <= wt <= 3.5):
        return 1, "velocidad de giro fuera de rango (0.2-3.0 / turbo 3.5)"
    s = open(TELEOP_YAML, encoding="utf-8").read()
    # y lleva signo NEGATIVO (calibracion real del mando); x positivo
    def rep(block, pairs, txt):
        def f(m):
            out = m.group(1)
            for i, val in enumerate(pairs):
                out += m.group(2 + i * 2) + str(val)
            return out
        pat = "(" + block + r":)"
        for k, _ in pairs_keys:
            pat += r"(\s*\n\s*" + k + r":\s*)(-?[\d.]+)"
        return re.sub(pat, f, txt, count=1)
    for block, pairs_keys, pairs in (
            ("scale_linear", [("x", 0), ("y", 0)], [v, -v]),
            ("scale_linear_turbo", [("x", 0), ("y", 0)], [vt, -vt]),
            ("scale_angular", [("yaw", 0)], [w]),
            ("scale_angular_turbo", [("yaw", 0)], [wt])):
        s = rep(block, pairs, s)
    open(TELEOP_YAML, "w", encoding="utf-8").write(s)
    code, out = rebuild_and_restart()
    return code, ("Velocidades aplicadas: normal %.2f m/s | turbo %.2f m/s | "
                  "giro %.2f | giro turbo %.2f rad/s\n%s" % (v, vt, w, wt, out))


SLAM_PRESETS = {
    # (min_travel_dist, min_travel_head, correlation_window)
    "normal": (0.2, 0.2, 0.5),
    "parquet": (0.1, 0.15, 1.0),
}


def slam_preset(name):
    if name not in SLAM_PRESETS:
        return 1, "preset desconocido: %r" % name
    d, h, c = SLAM_PRESETS[name]
    s = open(SLAM_YAML, encoding="utf-8").read()
    s = re.sub(r"(minimum_travel_distance:\s*)[\d.]+", r"\g<1>%s" % d, s)
    s = re.sub(r"(minimum_travel_heading:\s*)[\d.]+", r"\g<1>%s" % h, s)
    s = re.sub(r"(correlation_search_space_dimension:\s*)[\d.]+", r"\g<1>%s" % c, s)
    open(SLAM_YAML, "w", encoding="utf-8").write(s)
    code, out = rebuild_and_restart()
    return code, ("Preset SLAM '%s' (scan cada %sm/%srad, ventana %sm). El mapa "
                  "se resetea con el reinicio.\n%s" % (name, d, h, c, out))


def maps_list():
    items = []
    active = get_active_map()
    if os.path.isdir(MAPS_DIR):
        for f in sorted(os.listdir(MAPS_DIR)):
            if f.endswith(".yaml") and f != "dummy.yaml":
                name = f[:-5]
                pgm = os.path.join(MAPS_DIR, name + ".pgm")
                kb = os.path.getsize(pgm) // 1024 if os.path.exists(pgm) else 0
                items.append({"name": name, "kb": kb,
                              "activo": os.path.join(MAPS_DIR, f) == active})
    return items


def decode_throttled():
    _, t = sh("vcgencmd get_throttled 2>/dev/null | cut -d= -f2", 5)
    try:
        v = int(t, 16)
    except ValueError:
        return "?"
    now = "⚠️ BAJA TENSION AHORA" if v & 1 else "tension OK ahora"
    hist = " (hubo bajadas desde el arranque)" if v & 0x10000 else ""
    return now + hist


def status():
    _, active = sh("systemctl is-active robot-bringup.service", 10)
    _, load = sh("cut -d' ' -f1-3 /proc/loadavg", 5)
    _, joy = sh("test -e /dev/input/js0 && echo conectado || echo AUSENTE", 5)
    _, temp = sh("vcgencmd measure_temp 2>/dev/null | cut -d= -f2", 5)
    _, upt = sh("uptime -p", 5)
    sp = get_speeds()
    return {"servicio": active, "modo": MODES.get(get_mode(), "?"),
            "mapa_activo": os.path.basename(get_active_map()),
            "carga": load, "temperatura": temp, "alimentacion": decode_throttled(),
            "mando_js0": joy, "uptime": upt,
            "velocidades": "normal %.2f | turbo %.2f | giro %.2f | giro turbo %.2f"
                           % (sp["v"], sp["vt"], sp["w"], sp["wt"])}


def pgm_to_png(path, max_dim=900):
    """Convierte un PGM P5 (mapa de nav2) a PNG en gris, con submuestreo."""
    with open(path, "rb") as f:
        data = f.read()
    if not data.startswith(b"P5"):
        raise ValueError("no es PGM binario (P5)")
    # cabecera: P5, comentarios #, ancho alto, maxval
    pos, fields = 2, []
    while len(fields) < 3:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            while data[pos:pos + 1] not in (b"\n", b""):
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        fields.append(int(data[start:pos]))
    pos += 1  # el unico whitespace tras maxval
    w, h, _maxv = fields
    px = data[pos:pos + w * h]
    step = max(1, (max(w, h) + max_dim - 1) // max_dim)
    ow, oh = (w + step - 1) // step, (h + step - 1) // step
    raw = bytearray()
    for y in range(0, h, step):
        raw.append(0)  # filtro PNG: None
        row = px[y * w:(y + 1) * w]
        raw.extend(row[::step])
    def chunk(tag, payload):
        c = struct.pack(">I", len(payload)) + tag + payload
        return c + struct.pack(">I", zlib.crc32(tag + payload) & 0xffffffff)
    ihdr = struct.pack(">IIBBBBB", ow, oh, 8, 0, 0, 0, 0)  # gris 8 bits
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(bytes(raw), 6)) + chunk(b"IEND", b""))


def do_action(action, arg):
    if action == "status":
        return 0, json.dumps(status(), ensure_ascii=False, indent=1)
    if action == "log":
        return sh("tail -30 %s" % LOG_FILE, 10)
    if action == "reset_map":
        return sh("sudo -n systemctl restart robot-bringup.service && "
                  "echo 'Reiniciado: mapa en blanco. Espera ~25s y reconecta Foxglove.'", 90)
    if action == "set_mode":
        if arg not in MODES:
            return 1, "modo invalido: %r" % arg
        open(MODE_FILE, "w").write(arg + "\n")
        code, out = sh("sudo -n systemctl restart robot-bringup.service", 90)
        return code, "Modo '%s' aplicado. Espera ~25s y reconecta Foxglove.\n%s" % (MODES[arg], out)
    if action == "save_map":
        name = "".join(c for c in (arg or "casa") if c.isalnum() or c in "-_") or "casa"
        return sh("mkdir -p %s && source /opt/ros/jazzy/setup.bash && "
                  "source %s/install/setup.bash && "
                  "timeout 40 ros2 run nav2_map_server map_saver_cli -f %s/%s 2>&1 | tail -3 && "
                  "ls -la %s/%s.*" % (MAPS_DIR, WS, MAPS_DIR, name, MAPS_DIR, name), 60)
    if action == "stop_robot":
        return sh("sudo -n systemctl stop robot-bringup.service && echo 'Robot PARADO.'", 60)
    if action == "set_speeds":
        try:
            p = json.loads(arg)
            return set_speeds(p["v"], p["vt"], p["w"], p["wt"])
        except (KeyError, ValueError) as e:
            return 1, "parametros de velocidad invalidos: %s" % e
    if action == "slam_preset":
        return slam_preset(arg)
    if action == "maps_list":
        return 0, json.dumps(maps_list(), ensure_ascii=False, indent=1)
    if action == "set_active_map":
        y = os.path.join(MAPS_DIR, arg + ".yaml")
        if not os.path.exists(y):
            return 1, "no existe %s" % y
        open(MAP_FILE, "w").write(y + "\n")
        return 0, ("Mapa activo para navegacion: %s. Se usara la proxima vez que "
                   "entres (o reinicies) el modo Navegación." % arg)
    if action == "delete_map":
        y = os.path.join(MAPS_DIR, arg + ".yaml")
        if not os.path.exists(y):
            return 1, "no existe %s" % y
        if y == get_active_map():
            return 1, "es el mapa ACTIVO de navegacion; activa otro antes de borrarlo"
        return sh("rm -f %s/%s.yaml %s/%s.pgm && echo 'Mapa %s borrado.'"
                  % (MAPS_DIR, arg, MAPS_DIR, arg, arg), 10)
    if action == "motor_test":
        return sh("source /opt/ros/jazzy/setup.bash && source %s/install/setup.bash && "
                  "timeout 20 python3 /home/mimavi/motor_twitch.py" % WS, 30)
    if action == "joy_test":
        return sh("source /opt/ros/jazzy/setup.bash && "
                  "timeout 12 python3 /home/mimavi/joy_test.py" % (), 20)
    if action == "restart_eyes":
        return sh("sudo -n systemctl restart robot_eyes.service && echo 'Ojos reiniciados.'", 60)
    if action == "reboot_pi":
        sh("sudo -n shutdown -r +0 'reinicio desde el panel' &", 5)
        return 0, "Reiniciando el Pi... vuelve a cargar esta pagina en ~1 minuto."
    if action == "shutdown_pi":
        sh("sudo -n shutdown -h +0 'apagado desde el panel' &", 5)
        return 0, "Apagando el Pi de forma SEGURA. Puedes cortar la corriente cuando el LED verde deje de parpadear."
    return 1, "accion desconocida: %r" % action


PAGE = """<!DOCTYPE html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot — Panel de control</title>
<style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:16px;max-width:640px;margin:auto}
 h1{font-size:1.3em}
 button{display:block;width:100%;margin:8px 0;padding:13px;font-size:1.02em;border:0;border-radius:10px;cursor:pointer;font-weight:600}
 .b-map{background:#2d6cdf;color:#fff}.b-nav{background:#7a3fd1;color:#fff}
 .b-reset{background:#e67e22;color:#fff}.b-save{background:#27ae60;color:#fff}
 .b-stop{background:#c0392b;color:#fff}.b-info{background:#333;color:#eee}
 .b-warn{background:#8e5b00;color:#fff}
 pre{background:#000;border:1px solid #333;border-radius:8px;padding:10px;white-space:pre-wrap;font-size:.85em;min-height:80px}
 input,select{width:100%;padding:9px;border-radius:8px;border:1px solid #444;background:#222;color:#eee;box-sizing:border-box;margin:4px 0}
 details{border:1px solid #333;border-radius:10px;padding:8px 12px;margin:10px 0;background:#1a1a1a}
 summary{cursor:pointer;font-weight:700;padding:6px 0;font-size:1.05em}
 .row{display:flex;gap:8px}.row>*{flex:1}
 label{font-size:.85em;color:#aaa}
 .busy{opacity:.5;pointer-events:none}
 img{max-width:100%;border-radius:8px;border:1px solid #333;margin-top:8px}
 small{color:#888}
</style></head><body>
<h1>🤖 Panel del robot</h1>
<div id="wrap">

<button class="b-reset" onclick="act('reset_map')">🔄 Resetear mapa (reinicia modo actual)</button>
<button class="b-info" onclick="act('status')">📊 Estado</button>

<details open><summary>🚗 Modo del robot</summary>
 <button class="b-map" onclick="act('set_mode','mapping')">🗺️ MAPEO (SLAM + mando)</button>
 <button class="b-nav" onclick="act('set_mode','navigation')">🧭 NAVEGACIÓN (Nav2 + mapa activo)</button>
</details>

<details><summary>🎮 Velocidades del mando</summary>
 <div class="row"><div><label>Normal (m/s)</label><input id="v" value="0.40"></div>
 <div><label>Turbo (m/s)</label><input id="vt" value="0.80"></div></div>
 <div class="row"><div><label>Giro (rad/s)</label><input id="w" value="1.20"></div>
 <div><label>Giro turbo</label><input id="wt" value="2.00"></div></div>
 <button class="b-save" onclick="act('set_speeds',JSON.stringify({v:val('v'),vt:val('vt'),w:val('w'),wt:val('wt')}))">Aplicar velocidades (reinicia)</button>
 <small>Menos velocidad = menos patinaje = mejor mapa.</small>
</details>

<details><summary>🗺️ Ajuste de SLAM (presets)</summary>
 <button class="b-info" onclick="act('slam_preset','parquet')">🪵 Parquet resbaladizo (scans frecuentes + ventana ancha)</button>
 <button class="b-info" onclick="act('slam_preset','normal')">✳️ Normal (suelo con agarre)</button>
 <small>Cambiar el preset resetea el mapa.</small>
</details>

<details><summary>💾 Mapas</summary>
 <input id="mapname" placeholder="nombre del mapa (por defecto: casa)">
 <button class="b-save" onclick="act('save_map',document.getElementById('mapname').value)">💾 Guardar mapa actual</button>
 <button class="b-info" onclick="listMaps()">📃 Listar mapas</button>
 <div id="maps"></div>
 <div id="mapimg"></div>
</details>

<details><summary>🔧 Mantenimiento</summary>
 <button class="b-info" onclick="act('joy_test')">🎮 Probar mando (pulsa botones al lanzarlo)</button>
 <button class="b-warn" onclick="if(confirm('El robot girará un poco sobre sí mismo. ¿Suelo despejado?'))act('motor_test')">⚙️ Probar motores (giro de 1s)</button>
 <button class="b-info" onclick="act('restart_eyes')">👀 Reiniciar ojos</button>
 <button class="b-info" onclick="act('log')">📜 Log del robot</button>
 <button class="b-warn" onclick="if(confirm('¿Reiniciar la Raspberry?'))act('reboot_pi')">♻️ Reiniciar el Pi</button>
 <button class="b-stop" onclick="if(confirm('¿APAGAR la Raspberry de forma segura?'))act('shutdown_pi')">⏻ Apagar el Pi (seguro para la SSD)</button>
 <button class="b-stop" onclick="if(confirm('¿Parar TODO el robot?'))act('stop_robot')">⛔ Parar robot</button>
</details>

</div>
<pre id="out">Pulsa un botón…</pre>
<small>Foxglove: ws://192.168.1.233:8765 — tras reset/cambio de modo/velocidades, reconecta.</small>
<script>
function val(id){return parseFloat(document.getElementById(id).value)}
async function act(a, arg){
 const w=document.getElementById('wrap'), o=document.getElementById('out');
 w.classList.add('busy'); o.textContent='⏳ '+a+' …';
 try{
  const r=await fetch('/action',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({action:a,arg:arg||''})});
  const j=await r.json();
  o.textContent=(j.ok?'✅ ':'❌ ')+j.out;
  if(a==='status'&&j.ok){try{
    const s=JSON.parse(j.out), sp=s.velocidades.match(/[\\d.]+/g);
    if(sp){['v','vt','w','wt'].forEach((id,i)=>document.getElementById(id).value=sp[i]);}
  }catch(e){}}
 }catch(e){o.textContent='❌ error de red: '+e;}
 w.classList.remove('busy');
}
async function listMaps(){
 const d=document.getElementById('maps'), o=document.getElementById('out');
 o.textContent='⏳ listando…';
 const r=await fetch('/action',{method:'POST',headers:{'Content-Type':'application/json'},
   body:JSON.stringify({action:'maps_list',arg:''})});
 const j=await r.json();
 if(!j.ok){o.textContent='❌ '+j.out;return}
 const maps=JSON.parse(j.out); o.textContent='✅ '+maps.length+' mapa(s)';
 d.innerHTML=maps.map(m=>`<div class="row" style="align-items:center;margin:4px 0">
   <span style="flex:2">${m.activo?'⭐':'·'} <b>${m.name}</b> <small>${m.kb} KB</small></span>
   <button class="b-info" style="margin:0" onclick="showMap('${m.name}')">👁️</button>
   <button class="b-nav" style="margin:0" onclick="act('set_active_map','${m.name}').then(listMaps)">Usar</button>
   <button class="b-stop" style="margin:0" onclick="if(confirm('¿Borrar ${m.name}?'))act('delete_map','${m.name}').then(listMaps)">🗑️</button>
  </div>`).join('')||'<small>No hay mapas guardados.</small>';
}
function showMap(n){
 document.getElementById('mapimg').innerHTML='<img src="/map.png?name='+n+'&t='+Date.now()+'">';
}
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        data = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path in ("/", "/index.html"):
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif u.path == "/map.png":
            name = "".join(c for c in parse_qs(u.query).get("name", ["casa"])[0]
                           if c.isalnum() or c in "-_")
            pgm = os.path.join(MAPS_DIR, name + ".pgm")
            try:
                self._send(200, pgm_to_png(pgm), "image/png")
            except Exception as e:  # noqa: BLE001
                self._send(404, "error: %s" % e, "text/plain; charset=utf-8")
        else:
            self._send(404, "no existe", "text/plain; charset=utf-8")

    def do_POST(self):
        if self.path != "/action":
            self._send(404, "no existe", "text/plain; charset=utf-8")
            return
        try:
            n = int(self.headers.get("Content-Length", 0))
            req = json.loads(self.rfile.read(n) or b"{}")
            with _lock:
                code, out = do_action(req.get("action", ""), req.get("arg", ""))
            self._send(200, json.dumps({"ok": code == 0, "out": out},
                                       ensure_ascii=False), "application/json; charset=utf-8")
        except Exception as e:  # noqa: BLE001 — el panel nunca debe caerse
            self._send(200, json.dumps({"ok": False, "out": "error: %s" % e},
                                       ensure_ascii=False), "application/json; charset=utf-8")


if __name__ == "__main__":
    ThreadingHTTPServer(("0.0.0.0", 8080), Handler).serve_forever()
