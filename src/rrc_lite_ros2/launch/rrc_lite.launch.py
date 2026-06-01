"""
rrc_lite.launch.py
Lanza el driver de la placa Huaner/Hiwonder STM32 (RRC Lite).
Publica: /odom_raw, /imu/data_raw, /battery_voltage, TF odom→base_footprint
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rrc_lite_ros2'),
        'config', 'rrc_lite_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='rrc_lite_ros2',
            executable='rrc_lite_node',
            name='rrc_lite_node',
            output='screen',
            parameters=[config],
            # Remap si quieres cambiar nombres de topics
            # remappings=[('/cmd_vel', '/robot/cmd_vel')],
        ),
    ])
