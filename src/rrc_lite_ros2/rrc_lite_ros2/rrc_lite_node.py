#!/usr/bin/env python3
"""
rrc_lite_node.py
Nodo ROS 2 principal del driver RRC Lite (Hiwonder).

Suscripciones:
    /cmd_vel           (geometry_msgs/TwistStamped)  — velocidad deseada del robot
    /gimbal/cmd        (geometry_msgs/Vector3)   — pan/tilt del gimbal (rad)

Publicaciones:
    /odom_raw          (nav_msgs/Odometry)        — odometría por integración
    /imu/data_raw      (sensor_msgs/Imu)          — datos IMU (sin fusión)
    /battery_voltage   (std_msgs/Float32)         — tensión de batería (V)
    /tf                                           — odom → base_footprint

Parámetros:
    device         (string)  /dev/ttyACM0
    baudrate       (int)     1000000
    wheelbase      (float)   0.148   # MEDIR en el robot real
    track_width    (float)   0.140   # MEDIR en el robot real
    wheel_radius   (float)   0.033   # MEDIR en el robot real
    rps_calib      (float)   4.5     # el firmware da ~22% -> compensar x4.5
    body_reversed  (bool)    True    # robot montado girado 180 (trasera=frente)
    max_linear_vel (float)   0.25    m/s  (limitado para no saturar el motor)
    max_angular_vel(float)   1.5     rad/s
    gimbal_pan_id  (int)     1       servo PWM id para pan
    gimbal_tilt_id (int)     2       servo PWM id para tilt
    pan_center     (int)     1500    µs posición central pan
    tilt_center    (int)     1500    µs posición central tilt
    pan_range      (int)     500     µs — rango ±(pan_center ± pan_range)
    tilt_range     (int)     400     µs — rango ±
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist, TwistStamped, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from tf2_ros import TransformBroadcaster

from rrc_lite_ros2.hiwonder_board import HiwonderBoard
from rrc_lite_ros2.mecanum_kinematics import MecanumKinematics

# Covarianzas estándar para odometría mecanum (ajustar tras calibración)
_ODOM_POSE_COV = [
    1e-3, 0, 0, 0, 0, 0,
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e3,
]
_ODOM_TWIST_COV = [
    1e-3, 0, 0, 0, 0, 0,
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e3,
]
_G = 9.80665  # m/s²


class RrcLiteNode(Node):

    def __init__(self):
        super().__init__('rrc_lite_node')

        # ── Declarar parámetros ──────────────────────────────────────────────
        self.declare_parameter('device',          '/dev/ttyACM0')
        self.declare_parameter('baudrate',        1_000_000)
        self.declare_parameter('wheelbase',       0.148)
        self.declare_parameter('track_width',     0.140)
        self.declare_parameter('wheel_radius',    0.033)
        self.declare_parameter('rps_calib',       4.5)
        self.declare_parameter('body_reversed',   True)
        self.declare_parameter('max_linear_vel',  0.25)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('gimbal_pan_id',   1)
        self.declare_parameter('gimbal_tilt_id',  2)
        self.declare_parameter('pan_center',      1500)
        self.declare_parameter('tilt_center',     1500)
        self.declare_parameter('pan_range',       500)
        self.declare_parameter('tilt_range',      400)
        self.declare_parameter('odom_frame',      'odom')
        self.declare_parameter('base_frame',      'base_footprint')
        self.declare_parameter('imu_frame',       'imu_link')
        # Si hay EKF (robot_localization) publicando odom->base, este nodo NO
        # debe publicar tambien esa TF (doble publicador = TF inestable).
        self.declare_parameter('publish_odom_tf', True)

        device      = self.get_parameter('device').value
        baudrate    = self.get_parameter('baudrate').value
        wheelbase   = self.get_parameter('wheelbase').value
        track_width = self.get_parameter('track_width').value
        wheel_rad   = self.get_parameter('wheel_radius').value
        rps_calib   = self.get_parameter('rps_calib').value
        body_rev    = self.get_parameter('body_reversed').value
        self._max_v = self.get_parameter('max_linear_vel').value
        self._max_w = self.get_parameter('max_angular_vel').value
        self._pan_id    = self.get_parameter('gimbal_pan_id').value
        self._tilt_id   = self.get_parameter('gimbal_tilt_id').value
        self._pan_c     = self.get_parameter('pan_center').value
        self._tilt_c    = self.get_parameter('tilt_center').value
        self._pan_r     = self.get_parameter('pan_range').value
        self._tilt_r    = self.get_parameter('tilt_range').value
        self._odom_fr   = self.get_parameter('odom_frame').value
        self._base_fr   = self.get_parameter('base_frame').value
        self._imu_fr    = self.get_parameter('imu_frame').value
        self._publish_odom_tf = self.get_parameter('publish_odom_tf').value

        # ── Hardware ─────────────────────────────────────────────────────────
        self.get_logger().info(f'Conectando a placa Hiwonder en {device}...')
        try:
            self._board = HiwonderBoard(device, baudrate)
            self._board.enable_reception()
            self._board.set_buzzer(1800, 0.05, 0.01, 2)   # beep de arranque
            self.get_logger().info('✅ Placa Hiwonder conectada')
        except Exception as e:
            self.get_logger().error(f'❌ Error al conectar con la placa: {e}')
            raise

        self._kinematics = MecanumKinematics(wheelbase, track_width, wheel_rad,
                                             rps_calib=rps_calib,
                                             body_reversed=body_rev)
        self.get_logger().info(
            f'Cinematica: rps_calib={rps_calib}  body_reversed={body_rev}  '
            f'max_v={self._max_v}  max_w={self._max_w}')

        # ── Estado odometría ──────────────────────────────────────────────────
        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0
        self._vx  = 0.0
        self._vy  = 0.0
        self._wz  = 0.0
        self._last_cmd_time = time.time()

        # ── TF broadcaster ────────────────────────────────────────────────────
        self._tf_pub = TransformBroadcaster(self)

        # ── Publicadores ──────────────────────────────────────────────────────
        self._odom_pub    = self.create_publisher(Odometry, 'odom_raw',       5)
        self._imu_pub     = self.create_publisher(Imu,      'imu/data_raw',   5)
        self._battery_pub = self.create_publisher(Float32,  'battery_voltage', 1)

        # ── Suscriptores ──────────────────────────────────────────────────────
        self.create_subscription(TwistStamped, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Vector3, 'gimbal/cmd', self._on_gimbal,   10)

        # ── Timers ───────────────────────────────────────────────────────────
        self._dt = 0.02   # 50 Hz
        self.create_timer(self._dt,  self._pub_imu_odom)
        self.create_timer(1.0,       self._pub_battery)
        self.create_timer(0.5,       self._watchdog)   # para si no llegan comandos

        self.get_logger().info('rrc_lite_node iniciado ✅')

    # ── Callbacks de suscripción ──────────────────────────────────────────────
    def _on_cmd_vel(self, msg: TwistStamped):
        """Recibe TwistStamped, limita velocidades, envía a motores."""
        vx = max(-self._max_v, min(self._max_v, msg.twist.linear.x))
        vy = max(-self._max_v, min(self._max_v, msg.twist.linear.y))
        wz = max(-self._max_w, min(self._max_w, msg.twist.angular.z))

        self._vx = vx
        self._vy = vy
        self._wz = wz
        self._last_cmd_time = time.time()

        speeds = self._kinematics.cmd_vel_to_motor_rps(vx, vy, wz)
        self._board.set_motor_speed([[mid, rps] for mid, rps in speeds])

    def _on_gimbal(self, msg: Vector3):
        """
        Mueve el gimbal.
        msg.x = pan  (rad)  — se convierte a µs
        msg.y = tilt (rad)  — se convierte a µs
        """
        pan_us  = int(self._pan_c  + (msg.x / math.pi) * self._pan_r)
        tilt_us = int(self._tilt_c + (msg.y / math.pi) * self._tilt_r)
        pan_us  = max(self._pan_c  - self._pan_r,  min(self._pan_c  + self._pan_r,  pan_us))
        tilt_us = max(self._tilt_c - self._tilt_r, min(self._tilt_c + self._tilt_r, tilt_us))
        self._board.pwm_servo_set_position(0.1, [
            [self._pan_id,  pan_us],
            [self._tilt_id, tilt_us],
        ])

    # ── Publicación periódica ─────────────────────────────────────────────────
    def _pub_imu_odom(self):
        now = self.get_clock().now().to_msg()

        # ── IMU ──────────────────────────────────────────────────────────────
        imu_data = self._board.get_imu()
        if imu_data is not None:
            ax, ay, az, gx, gy, gz = imu_data
            imu_msg = Imu()
            imu_msg.header.stamp    = now
            imu_msg.header.frame_id = self._imu_fr
            imu_msg.linear_acceleration.x = ax * _G
            imu_msg.linear_acceleration.y = ay * _G
            imu_msg.linear_acceleration.z = az * _G
            imu_msg.angular_velocity.x = math.radians(gx)
            imu_msg.angular_velocity.y = math.radians(gy)
            imu_msg.angular_velocity.z = math.radians(gz)
            imu_msg.orientation_covariance[0] = -1.0  # orientación no disponible
            imu_msg.angular_velocity_covariance = [
                0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            imu_msg.linear_acceleration_covariance = [
                0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.004]
            self._imu_pub.publish(imu_msg)

        # ── Odometría por integración de velocidad ───────────────────────────
        delta_x   = (self._vx * math.cos(self._yaw) -
                     self._vy * math.sin(self._yaw)) * self._dt
        delta_y   = (self._vx * math.sin(self._yaw) +
                     self._vy * math.cos(self._yaw)) * self._dt
        delta_yaw = self._wz * self._dt

        self._x   += delta_x
        self._y   += delta_y
        self._yaw += delta_yaw

        # Quaternion (solo yaw)
        qz = math.sin(self._yaw / 2.0)
        qw = math.cos(self._yaw / 2.0)

        # TF
        tf = TransformStamped()
        tf.header.stamp       = now
        tf.header.frame_id    = self._odom_fr
        tf.child_frame_id     = self._base_fr
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        if self._publish_odom_tf:
            self._tf_pub.sendTransform(tf)

        # Odometry msg
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self._odom_fr
        odom.child_frame_id  = self._base_fr
        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x    = self._vx
        odom.twist.twist.linear.y    = self._vy
        odom.twist.twist.angular.z   = self._wz
        odom.pose.covariance  = _ODOM_POSE_COV
        odom.twist.covariance = _ODOM_TWIST_COV
        self._odom_pub.publish(odom)

    def _pub_battery(self):
        mv = self._board.get_battery()
        if mv is not None:
            msg = Float32()
            msg.data = mv / 1000.0   # mV → V
            self._battery_pub.publish(msg)

    def _watchdog(self):
        """Para los motores si no llegan comandos en 0.5 s."""
        if time.time() - self._last_cmd_time > 0.5:
            if self._vx != 0 or self._vy != 0 or self._wz != 0:
                self._vx = self._vy = self._wz = 0.0
                self._board.stop_motors()

    def destroy_node(self):
        self._board.stop_motors()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RrcLiteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
