import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import Joy
rclpy.init()
n = Node('joy_test_panel')
c = [0]; ev = []
base = [None]
def cb(m):
    c[0] += 1
    cur = ([round(a,1) for a in m.axes], list(m.buttons))
    if base[0] and cur != base[0] and len(ev) < 10:
        ev.append('ejes=%s botones=%s' % tuple(cur))
    base[0] = cur
n.create_subscription(Joy, '/joy', cb, 10)
t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.2)
if c[0] == 0:
    print('SIN DATOS del mando: dormido/dongle caido. Pulsa un boton, espera 5s y reintenta.')
else:
    print('OK: %d mensajes en 8s.' % c[0])
    print('\n'.join(ev) if ev else '(sin pulsaciones durante la prueba)')
