#!/usr/bin/env python3
"""
huskylens.launch.py — Lanzador del nodo HuskyLens 2

Uso independiente:
  ros2 launch huskylens2_ros2 huskylens.launch.py
  ros2 launch huskylens2_ros2 huskylens.launch.py algorithm:=face
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('huskylens2_ros2')
    config  = os.path.join(pkg_dir, 'config', 'huskylens_params.yaml')

    algorithm_arg = DeclareLaunchArgument(
        'algorithm', default_value='object_recognition',
        description='Algoritmo inicial de la HuskyLens 2. '
                    'Valores: face, object, tracking, line, color, tag, '
                    'gesture, pose, hand, ocr, qr, barcode')
    poll_rate_arg = DeclareLaunchArgument(
        'poll_rate', default_value='10.0',
        description='Frecuencia de lectura I2C en Hz')

    huskylens_node = Node(
        package='huskylens2_ros2',
        executable='huskylens_node',
        name='huskylens_node',
        output='screen',
        parameters=[
            config,
            {
                'algorithm':  LaunchConfiguration('algorithm'),
                'poll_rate':  LaunchConfiguration('poll_rate'),
            },
        ],
    )

    return LaunchDescription([
        algorithm_arg,
        poll_rate_arg,
        huskylens_node,
    ])
