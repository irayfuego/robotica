#!/usr/bin/env python3
"""
battery_alert_node.py -- aviso de bateria baja por los ojos + voz.

Se suscribe a /battery_voltage (std_msgs/Float32, voltios) que publica el
rrc_lite_node (lee la bateria de la placa RRC Lite). Cuando el nivel estimado
baja de los umbrales, avisa de forma TEMPORAL: pone cara de cansancio en los
ojos y dice por voz el porcentaje que queda, para poder enchufar a tiempo.

No abre el puerto serie (lo hace el rrc_lite_node): asi no compite por
/dev/ttyACM0 ni interfiere con el control de motores.

Subscriptions:
  /battery_voltage     (std_msgs/Float32)  -- tension de bateria en voltios

Publications:
  /robot/say           (std_msgs/String)   -- aviso hablado con el porcentaje
  /robot_eyes/emotion  (std_msgs/String)   -- expresion de alerta (cansancio)
  /robot/battery_pct   (std_msgs/Float32)  -- porcentaje estimado (telemetria)

El porcentaje se estima con una curva LiPo por celda escalada por 'cells'
(3S por defecto: la RRC admite 5-12.6 V). Si tu bateria es otra, ajusta
'battery_cells' o los umbrales en el YAML.
"""

import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float32
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print('[WARN] rclpy not found -- battery_alert_node needs ROS to run.')


# Curva de descarga LiPo POR CELDA (voltios/celda -> % restante), interpolada.
_LIPO_CELL = [
    (4.20, 100), (4.15, 90), (4.11, 80), (4.08, 70), (4.02, 60),
    (3.95, 50), (3.91, 40), (3.88, 30), (3.84, 20), (3.77, 10), (3.50, 0),
]


def _cell_to_pct(vc):
    """Voltios por celda -> porcentaje (0-100) por interpolacion lineal."""
    if vc >= _LIPO_CELL[0][0]:
        return 100
    if vc <= _LIPO_CELL[-1][0]:
        return 0
    for (v1, p1), (v2, p2) in zip(_LIPO_CELL, _LIPO_CELL[1:]):
        if v2 <= vc <= v1:
            return int(round(p2 + (p1 - p2) * (vc - v2) / (v1 - v2)))
    return 0


_SEVERITY = {'warn': 1, 'low': 2, 'critical': 3}


class BatteryAlertNode(Node if HAS_ROS else object):

    def __init__(self):
        super().__init__('battery_alert_node')

        self.declare_parameter('battery_cells',   3)      # 3S por defecto
        self.declare_parameter('warn_pct',        30)     # primer aviso suave
        self.declare_parameter('low_pct',         15)     # enchufar ya
        self.declare_parameter('critical_pct',    7)      # apagado inminente
        self.declare_parameter('reannounce_sec',  180.0)  # repetir si sigue bajo
        self.declare_parameter('min_valid_v',     5.0)    # por debajo = lectura falsa

        self._cells   = max(1, int(self.get_parameter('battery_cells').value))
        self._warn    = int(self.get_parameter('warn_pct').value)
        self._low     = int(self.get_parameter('low_pct').value)
        self._crit    = int(self.get_parameter('critical_pct').value)
        self._reann   = float(self.get_parameter('reannounce_sec').value)
        self._min_v   = float(self.get_parameter('min_valid_v').value)

        self._pub_say     = self.create_publisher(String,  '/robot/say',          10)
        self._pub_emotion = self.create_publisher(String,  '/robot_eyes/emotion', 10)
        self._pub_pct     = self.create_publisher(Float32, '/robot/battery_pct',  10)

        self.create_subscription(Float32, '/battery_voltage', self._cb_voltage, 10)

        self._last_level = None    # 'warn' | 'low' | 'critical' | None (ok)
        self._last_alert_t = 0.0
        self._ema_v = None         # voltaje suavizado (evita avisos por picos)

        self.get_logger().info(
            'Vigilante de bateria activo  %dS  umbrales: warn=%d%% low=%d%% crit=%d%%'
            % (self._cells, self._warn, self._low, self._crit))

    def _cb_voltage(self, msg):
        v = float(msg.data)
        if v < self._min_v:
            return   # 0 / lectura invalida (RRC sin bateria de potencia)
        # Suavizado exponencial: el voltaje fluctua con la carga del motor; sin
        # esto un pico bajo dispararia avisos falsos.
        self._ema_v = v if self._ema_v is None else 0.3 * v + 0.7 * self._ema_v
        pct = _cell_to_pct(self._ema_v / self._cells)

        pm = Float32(); pm.data = float(pct)
        self._pub_pct.publish(pm)

        level = None
        if pct <= self._crit:
            level = 'critical'
        elif pct <= self._low:
            level = 'low'
        elif pct <= self._warn:
            level = 'warn'

        if level is None:
            self._last_level = None    # recuperada (enchufada) -> rearma avisos
            return

        now = time.time()
        worse = (self._last_level is None
                 or _SEVERITY[level] > _SEVERITY.get(self._last_level, 0))
        stale = (now - self._last_alert_t) > self._reann
        if worse or stale:
            self._alert(level, pct)
            self._last_level = level
            self._last_alert_t = now

    def _alert(self, level, pct):
        if level == 'warn':
            txt = 'Bateria al %d por ciento.' % pct
            emo = 'tired'
        elif level == 'low':
            txt = 'Bateria baja, al %d por ciento. Enchufame, por favor.' % pct
            emo = 'tired'
        else:
            txt = ('Bateria muy baja, %d por ciento. Voy a quedarme sin energia, '
                   'enchufame ya.' % pct)
            emo = 'sad'
        em = String(); em.data = emo
        self._pub_emotion.publish(em)
        sm = String(); sm.data = txt
        self._pub_say.publish(sm)
        self.get_logger().warn('Bateria %d%% (%s) -> aviso' % (pct, level))


def main(args=None):
    if not HAS_ROS:
        return
    rclpy.init(args=args)
    node = BatteryAlertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
