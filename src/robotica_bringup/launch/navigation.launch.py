#!/usr/bin/env python3
"""
navigation.launch.py - MODO NAVEGACION (Opcion 2)

AMCL (localizacion sobre un mapa GUARDADO) + Nav2, base motriz, LIDAR y EKF.
SIN SLAM en vivo -> mucho mas ligero que el mapeo, asi Nav2 cabe en el Pi 2GB.

Requiere un mapa creado antes con mapping.launch.py y guardado:
  ros2 run nav2_map_server map_saver_cli -f /home/mimavi/maps/casa

Uso:
  ros2 launch robotica_bringup navigation.launch.py
  ros2 launch robotica_bringup navigation.launch.py map:=/home/mimavi/maps/otro.yaml

Tras arrancar, dar la POSE INICIAL del robot en Foxglove (2D Pose Estimate) o por
/initialpose; luego mandar un goal a /goal_pose. El override manual con el mando
funciona (teleop -> twist_mux, prioridad mayor que Nav2).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('robotica_bringup')
    rrc_dir     = get_package_share_directory('rrc_lite_ros2')
    lidar_dir   = get_package_share_directory('m1ct_d2')
    nav2_dir    = get_package_share_directory('nav2_bringup')

    cfg            = os.path.join(bringup_dir, 'config')
    ekf_yaml       = os.path.join(cfg, 'ekf.yaml')
    nav2_yaml      = os.path.join(cfg, 'nav2_params.yaml')
    twist_mux_yaml = os.path.join(cfg, 'twist_mux.yaml')

    map_arg = DeclareLaunchArgument(
        'map', default_value='/home/mimavi/maps/casa.yaml',
        description='Ruta al mapa (.yaml) guardado con map_saver_cli')
    map_path = LaunchConfiguration('map')

    # 1. Base motriz (recibe /cmd_vel, publica /odom_raw, /imu/data_raw)
    rrc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rrc_dir, 'launch', 'rrc_lite.launch.py')))

    # 2. LIDAR
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_dir, 'launch', 'lidar.launch.py')),
        launch_arguments={'port': '/dev/sc_mini'}.items())

    # 3. EKF: /odom_raw + giroscopio -> TF odom->base_footprint
    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_yaml], remappings=[('odometry/filtered', '/odom')])

    # 4. TFs estaticos
    base_laser_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_link_to_laser_link',
        arguments=['0', '0', '0.18', '-1.5708', '0', '0', 'base_link', 'laser_link'])
    base_footprint_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'])
    base_imu_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'])

    # 5. Localizacion: map_server + AMCL (publica TF map->odom)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_path,
            'params_file': nav2_yaml,
            'use_sim_time': 'false',
        }.items())

    # 6. Nav2 (controller, planner, bt_navigator, collision_monitor, ...)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_yaml,
            'use_sim_time': 'false',
            'use_lifecycle_mgr': 'true',
        }.items())

    # 7. twist_mux + teleop (override manual con el mando)
    twist_mux_node = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux',
        output='screen', parameters=[twist_mux_yaml])
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'teleop.launch.py')))

    # 8. Foxglove
    foxglove_bridge = Node(
        package='foxglove_bridge', executable='foxglove_bridge',
        name='foxglove_bridge', output='screen',
        parameters=[{'port': 8765, 'address': '0.0.0.0', 'tls': False,
                     'topic_whitelist': ['.*'], 'max_qos_depth': 10,
                     'num_threads': 2, 'send_buffer_limit': 10000000}])

    # Sin TimerAction: envolver el IncludeLaunchDescription de la localizacion en
    # un TimerAction rompia el paso del argumento 'map' (map_server arrancaba con
    # yaml_filename vacio). Los nodos esperan sus dependencias (TF, etc.) por si
    # mismos, asi que no hace falta escalonar el arranque.
    return LaunchDescription([
        map_arg,
        rrc_launch, lidar_launch,
        base_laser_tf, base_footprint_tf, base_imu_tf,
        ekf_node,
        twist_mux_node, teleop_launch, foxglove_bridge,
        localization_launch,
        nav2_launch,
    ])
