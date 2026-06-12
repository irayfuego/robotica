#!/usr/bin/env python3
"""
lidar_test.launch.py -- Modo TEST: ver el LIDAR en Foxglove sin la placa RRC.

Mientras NO este conectada la controladora de motores (RRC Lite), que es la
fuente de odometria real, el bringup normal (robot.launch.py) falla: el EKF no
recibe /odom_raw, no publica el TF odom -> base_footprint y se rompe toda la
cadena de TF (map -> odom -> base_footprint -> base_link -> laser_link). Por eso
Foxglove muestra "no coordinate frames found" aunque el LIDAR publique /scan.

Este launch arranca SOLO lo imprescindible para visualizar el LIDAR:
  - El driver del LIDAR real (m1ct_d2) publicando /scan en frame laser_link.
  - TODOS los TFs estaticos, incluidos los falsos que sustituyen a EKF y SLAM:
        map -> odom -> base_footprint -> base_link -> laser_link
    Asi la cadena queda completa y Foxglove encuentra los frames.
  - foxglove_bridge en el puerto 8765.

No lanza rrc_lite, EKF ni Nav2 (no tienen sentido sin la placa de motores).

Uso:
  ros2 launch robotica_bringup lidar_test.launch.py
  ros2 launch robotica_bringup lidar_test.launch.py slam:=true   # mapear inmovil
  ros2 launch robotica_bringup lidar_test.launch.py port:=/dev/ttyUSB0

Cuando llegue la placa RRC, volver al bringup normal:
  ros2 launch robotica_bringup robot.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('robotica_bringup')
    lidar_dir   = get_package_share_directory('m1ct_d2')
    slam_yaml   = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')

    # ── Argumentos ────────────────────────────────────────────────────────────
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/sc_mini',
        description='Puerto serie del LIDAR (symlink udev; fallback /dev/ttyUSB0)')
    slam_arg = DeclareLaunchArgument(
        'slam', default_value='false',
        description='true = lanza slam_toolbox (construye mapa con el robot inmovil)')

    port = LaunchConfiguration('port')
    slam = LaunchConfiguration('slam')

    # ── LIDAR real ────────────────────────────────────────────────────────────
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_dir, 'launch', 'lidar.launch.py')),
        launch_arguments={'port': port}.items(),
    )

    # ── TFs estaticos reales ──────────────────────────────────────────────────
    base_laser_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_link_to_laser_link',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser_link'])

    base_footprint_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'])

    # ── TFs falsos (sustituyen a EKF y SLAM mientras no haya odometria real) ───
    # odom -> base_footprint: lo daria el EKF; aqui es estatico (robot inmovil).
    fake_odom_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='test_odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'])

    # map -> odom: normalmente lo publica SLAM. Solo lo falseamos si SLAM esta
    # apagado (si no, habria DOS publicadores del mismo TF y entrarian en
    # conflicto).
    fake_map_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='test_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=UnlessCondition(slam))

    # ── SLAM opcional ─────────────────────────────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': slam_yaml}.items(),
        condition=IfCondition(slam))

    # ── Foxglove Bridge ───────────────────────────────────────────────────────
    foxglove_bridge = Node(
        package='foxglove_bridge', executable='foxglove_bridge',
        name='foxglove_bridge', output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'topic_whitelist': ['.*'],
            'max_qos_depth': 10,
            'num_threads': 2,
            'send_buffer_limit': 10000000,
            'use_compression': False,
        }])

    return LaunchDescription([
        port_arg,
        slam_arg,
        lidar_launch,
        base_laser_tf,
        base_footprint_tf,
        fake_odom_tf,
        fake_map_tf,
        TimerAction(period=3.0, actions=[slam_launch]),
        foxglove_bridge,
    ])
