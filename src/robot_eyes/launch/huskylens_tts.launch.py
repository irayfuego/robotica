#!/usr/bin/env python3
"""
huskylens_tts.launch.py — HuskyLens 2 Wi-Fi + TTS
Lanza:
  - huskylens_wifi_node: lee detecciones de la cámara vía HTTP
  - tts_node:            reproduce texto por el altavoz de la Pi

Uso:
  ros2 launch robot_eyes huskylens_tts.launch.py
  ros2 launch robot_eyes huskylens_tts.launch.py host:=192.168.1.32 algorithm:=13
  ros2 topic pub /tts/say std_msgs/String "data: 'Hola mundo'"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('host',      default_value='192.168.1.32'),
        DeclareLaunchArgument('algorithm', default_value='13'),
        DeclareLaunchArgument('poll_rate', default_value='3.0'),
        DeclareLaunchArgument('tts_enabled', default_value='true'),

        Node(
            package='robot_eyes',
            executable='huskylens_wifi',
            name='huskylens_wifi_node',
            output='screen',
            parameters=[{
                'host':        LaunchConfiguration('host'),
                'algorithm':   LaunchConfiguration('algorithm'),
                'poll_rate':   LaunchConfiguration('poll_rate'),
                'tts_enabled': LaunchConfiguration('tts_enabled'),
                'frame_id':    'camera_link',
            }],
        ),

        Node(
            package='robot_eyes',
            executable='tts_node',
            name='tts_node',
            output='screen',
            parameters=[{
                'piper_model': '/home/mimavi/piper_models/es_ES-sharvard-medium.onnx',
                'piper_bin':   '/home/mimavi/.local/bin/piper',
                'audio_device': 'default',
                'volume':      90,
            }],
        ),
    ])
