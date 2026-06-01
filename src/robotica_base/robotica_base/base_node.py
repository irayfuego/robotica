#!/usr/bin/env python3
"""
Nodo base del robot autónomo.
Convierte cmd_vel (Twist) en comandos de motor para la placa RRC Lite.
Este nodo será completado cuando se instale el driver de la RRC Lite.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robotica_msgs.msg import MotorCommand


class BaseNode(Node):
    """Controlador diferencial de 4 ruedas con tracción independiente."""

    def __init__(self):
        super().__init__('base_node')

        # Parámetros del robot (ajusta a tu geometría real)
        self.declare_parameter('wheel_base', 0.15)      # metros entre ejes izq/der
        self.declare_parameter('wheel_radius', 0.033)   # metros
        self.declare_parameter('max_speed', 1.0)        # m/s máximo

        self.wheel_base   = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_speed    = self.get_parameter('max_speed').value

        # Suscriptor de velocidad deseada
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publicador de comandos de motor
        self.motor_pub = self.create_publisher(
            MotorCommand, 'motor_command', 10)

        self.get_logger().info('BaseNode iniciado. Escuchando /cmd_vel...')

    def cmd_vel_callback(self, msg: Twist):
        """Convierte Twist a comandos individuales de motor (modelo diferencial)."""
        v = msg.linear.x    # velocidad lineal (m/s)
        w = msg.angular.z   # velocidad angular (rad/s)

        # Cinemática diferencial
        v_left  = (v - w * self.wheel_base / 2.0) / self.max_speed
        v_right = (v + w * self.wheel_base / 2.0) / self.max_speed

        # Limitar a [-1, 1]
        v_left  = max(-1.0, min(1.0, v_left))
        v_right = max(-1.0, min(1.0, v_right))

        cmd = MotorCommand()
        cmd.front_left  = v_left
        cmd.rear_left   = v_left
        cmd.front_right = v_right
        cmd.rear_right  = v_right

        self.motor_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
