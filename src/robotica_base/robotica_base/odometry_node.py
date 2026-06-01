#!/usr/bin/env python3
"""
Nodo de odometría del robot.
Integra los datos de encoder para publicar la posición estimada del robot.
Será completado cuando se instale el driver de la RRC Lite.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from robotica_msgs.msg import EncoderData
import math


class OdometryNode(Node):
    """Calcula y publica la odometría del robot a partir de los encoders."""

    def __init__(self):
        super().__init__('odometry_node')

        self.declare_parameter('wheel_base',   0.15)
        self.declare_parameter('wheel_radius', 0.033)

        self.wheel_base   = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # Estado de la odometría
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.last_ticks = None

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publicador de odometría
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Suscriptor de encoders
        self.encoder_sub = self.create_subscription(
            EncoderData, 'encoder_data', self.encoder_callback, 10)

        self.get_logger().info('OdometryNode iniciado.')

    def encoder_callback(self, msg: EncoderData):
        """Integra los ticks de encoder para calcular la posición."""
        if self.last_ticks is None:
            self.last_ticks = msg.ticks
            return

        # Diferencia de ticks (promedio izq y der)
        d_left  = (msg.ticks[0] - self.last_ticks[0] +
                   msg.ticks[2] - self.last_ticks[2]) / 2.0
        d_right = (msg.ticks[1] - self.last_ticks[1] +
                   msg.ticks[3] - self.last_ticks[3]) / 2.0
        self.last_ticks = msg.ticks

        # Distancia en metros (ajusta ticks_per_rev a tu encoder)
        ticks_per_rev = 30.0  # típico para motores TT con encoder
        circumference = 2 * math.pi * self.wheel_radius
        d_l = (d_left  / ticks_per_rev) * circumference
        d_r = (d_right / ticks_per_rev) * circumference

        d     = (d_l + d_r) / 2.0
        delta = (d_r - d_l) / self.wheel_base

        self.x   += d * math.cos(self.yaw + delta / 2.0)
        self.y   += d * math.sin(self.yaw + delta / 2.0)
        self.yaw += delta

        # Publicar TF odom → base_footprint
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # Publicar Odometry
        odom = Odometry()
        odom.header.stamp    = t.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = t.transform.rotation.z
        odom.pose.pose.orientation.w = t.transform.rotation.w
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
