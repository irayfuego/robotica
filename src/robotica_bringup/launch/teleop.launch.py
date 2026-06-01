#!/usr/bin/env python3
"""
teleop.launch.py — Teleop con mando 8BitDo Ultimate 2C

Lanza joy_node + teleop_twist_joy para controlar el robot manualmente.
El mando debe estar emparejado por Bluetooth antes de lanzar.

Uso:
  ros2 launch robotica_bringup teleop.launch.py
  ros2 launch robotica_bringup teleop.launch.py device_id:=1

Emparejamiento previo (una sola vez):
  bluetoothctl
    power on
    agent on
    default-agent
    scan on
    pair   <MAC>
    trust  <MAC>
    connect <MAC>
    quit

Modo del mando: Android (Start+B al encender hasta que el LED parpadee rápido).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    cfg = os.path.join(
        get_package_share_directory('robotica_bringup'),
        'config', '8bitdo_teleop.yaml'
    )

    device_id_arg = DeclareLaunchArgument(
        'device_id', default_value='0',
        description='ID del dispositivo joystick (/dev/input/jsX o evento SDL)')

    device_id = LaunchConfiguration('device_id')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[cfg, {'device_id': device_id}],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[cfg],
        remappings=[
            # TwistStamped → /cmd_vel/teleop → twist_mux (prioridad 10)
            ('/cmd_vel', '/cmd_vel/teleop'),
        ],
    )

    return LaunchDescription([
        device_id_arg,
        joy_node,
        teleop_node,
    ])
