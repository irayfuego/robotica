#!/usr/bin/env python3
"""
lidar.launch.py — Driver COIN-D6 (Guoke Optoelectronics / CSPC m1ct_d2)

Lanza el nodo m1ct_d2 que publica sensor_msgs/LaserScan en /scan.

Requisitos previos en la Pi:
  sudo cp sc_mini.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger

Uso:
  ros2 launch m1ct_d2 lidar.launch.py
  ros2 launch m1ct_d2 lidar.launch.py port:=/dev/ttyUSB0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('m1ct_d2')
    params_file = os.path.join(pkg_dir, 'params', 'm1ct_d2.yaml')

    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/sc_mini',
        description='Puerto serial del LIDAR (p.ej. /dev/ttyUSB0 o /dev/sc_mini)')

    lidar_node = Node(
        package='m1ct_d2',
        executable='m1ct_d2',
        name='m1ct_d2_node',
        output='screen',
        parameters=[params_file, {'port': LaunchConfiguration('port')}],
        remappings=[
            ('scan', '/scan'),
        ],
    )

    return LaunchDescription([
        port_arg,
        lidar_node,
    ])
