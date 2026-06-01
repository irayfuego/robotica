#!/usr/bin/env python3
"""
robot_eyes.launch.py — Lanza el nodo de ojos animados

Uso:
  ros2 launch robot_eyes robot_eyes.launch.py
  ros2 launch robot_eyes robot_eyes.launch.py sim_mode:=true   # sin pantallas
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

    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode', default_value='false',
        description='true = modo simulación sin pantallas físicas')

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

    return LaunchDescription([
        sim_mode_arg,
        robot_eyes_node,
    ])
