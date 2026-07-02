import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
rclpy.init()
n = Node('motor_twitch_panel')
pub = n.create_publisher(TwistStamped, '/cmd_vel', 10)
time.sleep(2.0)
m = TwistStamped()
t0 = time.time()
while time.time() - t0 < 1.0:
    m.header.stamp = n.get_clock().now().to_msg()
    m.twist.angular.z = 0.5
    pub.publish(m); time.sleep(0.05)
m.twist.angular.z = 0.0
for _ in range(5):
    m.header.stamp = n.get_clock().now().to_msg()
    pub.publish(m); time.sleep(0.05)
print('Giro de 1s enviado (y parada). Si no se ha movido: revisa bateria RRC / modo actual.')
