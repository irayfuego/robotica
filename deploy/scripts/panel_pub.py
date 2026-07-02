"""Publicador puntual para el panel: say | emotion | iris r g b (0-1)."""
import sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA

cmd = sys.argv[1]
rclpy.init()
n = Node('panel_pub')
if cmd == 'say':
    pub = n.create_publisher(String, '/robot/say', 10)
    msg = String(); msg.data = sys.argv[2]
elif cmd == 'emotion':
    pub = n.create_publisher(String, '/robot_eyes/emotion', 10)
    msg = String(); msg.data = sys.argv[2]
elif cmd == 'iris':
    pub = n.create_publisher(ColorRGBA, '/robot_eyes/iris_color', 10)
    msg = ColorRGBA()
    msg.r, msg.g, msg.b, msg.a = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), 1.0
else:
    raise SystemExit('comando desconocido: %r' % cmd)
time.sleep(1.5)  # descubrimiento DDS
if pub.get_subscription_count() == 0:
    time.sleep(1.5)
if pub.get_subscription_count() == 0:
    print('AVISO: nadie suscrito (ojos apagados?). Publico igualmente.')
pub.publish(msg)
time.sleep(0.4)
print('publicado %s' % cmd)
