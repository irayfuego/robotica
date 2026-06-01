#!/usr/bin/env python3
"""
robot_eyes_huskylens.launch.py — Ojos + HuskyLens 2

Lanza el nodo de ojos y el bridge que traduce las detecciones
de la HuskyLens 2 (emociones, caras, objetos) en comportamientos
de los ojos.

Uso:
  ros2 launch robot_eyes robot_eyes_huskylens.launch.py
  ros2 launch robot_eyes robot_eyes_huskylens.launch.py mode:=0   # face tracking
  ros2 launch robot_eyes robot_eyes_huskylens.launch.py sim_mode:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_eyes')
    config  = os.path.join(pkg_dir, 'config', 'robot_eyes_params.yaml')

    # ── Argumentos ────────────────────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='6',
        description=(
            'Modo HuskyLens 2: '
            '0=face_tracking, 1=object_tracking, 2=object_recognition, '
            '3=line_tracking, 4=color_recognition, 5=tag_recognition, '
            '6=emotion_recognition (default)'))

    interface_arg = DeclareLaunchArgument(
        'interface', default_value='uart',
        description='Interfaz HuskyLens: uart | i2c')

    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode', default_value='false',
        description='true = sin pantallas físicas')

    # ── Nodo de ojos ──────────────────────────────────────────────────────────
    robot_eyes_node = Node(
        package='robot_eyes',
        executable='robot_eyes_node',
        name='robot_eyes_node',
        output='screen',
        parameters=[
            config,
            {'sim_mode': LaunchConfiguration('sim_mode')},
        ],
    )

    # ── Bridge HuskyLens → ojos ───────────────────────────────────────────────
    bridge_node = Node(
        package='robot_eyes',
        executable='huskylens_bridge',
        name='huskylens_bridge_node',
        output='screen',
        parameters=[
            config,
            {
                'mode':      LaunchConfiguration('mode'),
                'interface': LaunchConfiguration('interface'),
            },
        ],
    )

    return LaunchDescription([
        mode_arg,
        interface_arg,
        sim_mode_arg,
        robot_eyes_node,
        bridge_node,
    ])
