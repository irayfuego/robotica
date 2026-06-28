#!/usr/bin/env python3
"""
mapping.launch.py - MODO MAPEO (Opcion 2)

SLAM Toolbox + base motriz (rrc_lite) + LIDAR + teleop (mando 8BitDo) + Foxglove.
SIN Nav2 (que satura el Pi 2GB): asi SLAM tiene CPU para construir el mapa.

Uso:
  1) Parar el bringup completo:   sudo systemctl stop robot-bringup.service
  2) ros2 launch robotica_bringup mapping.launch.py
  3) Conducir con el mando (manten RB pulsado) para recorrer la zona.
  4) Guardar el mapa:
       mkdir -p /home/mimavi/maps
       ros2 run nav2_map_server map_saver_cli -f /home/mimavi/maps/casa

Odometria: la publica rrc_lite_node (odom->base_footprint). Sin EKF para
mantenerlo ligero; si el mapa sale distorsionado en los giros, anadir EKF/IMU.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('robotica_bringup')
    rrc_dir     = get_package_share_directory('rrc_lite_ros2')
    lidar_dir   = get_package_share_directory('m1ct_d2')
    slam_dir    = get_package_share_directory('slam_toolbox')

    cfg       = os.path.join(bringup_dir, 'config')
    slam_yaml = os.path.join(cfg, 'slam_toolbox.yaml')
    joy_yaml  = os.path.join(cfg, '8bitdo_teleop.yaml')
    ekf_yaml  = os.path.join(cfg, 'ekf.yaml')

    # 1. Motores: recibe /cmd_vel (TwistStamped), publica odom->base_footprint
    rrc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rrc_dir, 'launch', 'rrc_lite.launch.py')))

    # 2. LIDAR (publica /scan)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_dir, 'launch', 'lidar.launch.py')),
        launch_arguments={'port': '/dev/sc_mini'}.items())

    # 3. Teleop: joy_node + teleop_twist_joy -> /cmd_vel DIRECTO
    #    (sin twist_mux; en mapeo no hay Nav2 ni collision_monitor)
    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        output='screen', parameters=[joy_yaml, {'device_id': 0}])
    teleop_node = Node(
        package='teleop_twist_joy', executable='teleop_node',
        name='teleop_twist_joy_node', output='screen',
        parameters=[joy_yaml])   # publica TwistStamped en /cmd_vel por defecto

    # 4. TFs estaticos (base_footprint -> base_link -> laser_link)
    base_laser_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_link_to_laser_link',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser_link'])
    base_footprint_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'])
    # IMU en base_link (Z vertical, identidad sirve para el yaw del giroscopio)
    base_imu_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'])

    # 5. SLAM Toolbox (mode: mapping en el yaml)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': slam_yaml}.items())

    # 6. EKF: fusiona /odom_raw + IMU (giroscopio) -> TF odom->base_footprint
    #    con giro fiable. rrc_lite_node tiene publish_odom_tf=false para no chocar.
    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_yaml],
        remappings=[('odometry/filtered', '/odom')])

    # 7. Foxglove (para ver el mapa en construccion)
    foxglove_bridge = Node(
        package='foxglove_bridge', executable='foxglove_bridge',
        name='foxglove_bridge', output='screen',
        parameters=[{'port': 8765, 'address': '0.0.0.0', 'tls': False,
                     'topic_whitelist': ['.*'], 'max_qos_depth': 10,
                     'num_threads': 2, 'send_buffer_limit': 10000000}])

    return LaunchDescription([
        rrc_launch, lidar_launch,
        joy_node, teleop_node,
        base_laser_tf, base_footprint_tf, base_imu_tf,
        ekf_node, slam_launch, foxglove_bridge,
    ])
