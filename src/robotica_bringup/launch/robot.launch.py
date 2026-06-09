#!/usr/bin/env python3
"""
robot.launch.py -- Bringup completo del robot Mecanum

Uso:
  ros2 launch robotica_bringup robot.launch.py
  ros2 launch robotica_bringup robot.launch.py sim_mode:=true use_camera:=false
  ros2 launch robotica_bringup robot.launch.py slam:=false map:=/path/to/map.yaml
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

    bringup_dir  = get_package_share_directory('robotica_bringup')
    rrc_dir      = get_package_share_directory('rrc_lite_ros2')
    huskydir     = get_package_share_directory('huskylens2_ros2')
    nav2_dir     = get_package_share_directory('nav2_bringup')
    lidar_dir    = get_package_share_directory('m1ct_d2')

    cfg            = os.path.join(bringup_dir, 'config')
    ekf_yaml       = os.path.join(cfg, 'ekf.yaml')
    nav2_yaml      = os.path.join(cfg, 'nav2_params.yaml')
    slam_yaml      = os.path.join(cfg, 'slam_toolbox.yaml')
    twist_mux_yaml = os.path.join(cfg, 'twist_mux.yaml')

    # ── Argumentos ────────────────────────────────────────────────────────────
    slam_arg = DeclareLaunchArgument(
        'slam', default_value='true',
        description='true = modo SLAM; false = AMCL con mapa existente')
    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Ruta al mapa YAML (solo si slam:=false)')
    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Lanzar huskylens_node')
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode', default_value='false',
        description='true = sin hardware fisico (publica TFs estaticos falsos)')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Usar reloj de simulacion')

    slam         = LaunchConfiguration('slam')
    map_path     = LaunchConfiguration('map')
    use_camera   = LaunchConfiguration('use_camera')
    sim_mode     = LaunchConfiguration('sim_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── 1. Controlador motores (solo con hardware) ────────────────────────────
    rrc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rrc_dir, 'launch', 'rrc_lite.launch.py')),
        condition=UnlessCondition(sim_mode),
    )

    # ── 2. LIDAR COIN-D6 (solo con hardware) ──────────────────────────────────
    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_dir, 'launch', 'lidar.launch.py')),
        launch_arguments={
            'port': '/dev/sc_mini',   # udev symlink CH340; fallback: /dev/ttyUSB0
        }.items(),
        condition=UnlessCondition(sim_mode),
    )

    # ── 3. Camara HuskyLens 2 (opcional) ──────────────────────────────────────
    huskylens_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(huskydir, 'launch', 'huskylens.launch.py')),
        condition=IfCondition(use_camera),
    )

    # ── 4. EKF (solo con hardware real) ───────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odom')],
        condition=UnlessCondition(sim_mode),
    )

    # ── TFs estaticos (siempre) ────────────────────────────────────────────────
    # base_link → laser_link: posicion del LIDAR respecto al centro del robot
    # NOTA: el LIDAR publica frame_id='laser_link'; este TF debe usar ESE nombre.
    base_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_link',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser_link'],
    )

    # base_footprint → base_link (siempre; en hardware lo necesita el EKF también)
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
    )

    # ── TFs estaticos falsos (solo en sim_mode) ────────────────────────────────
    # Sin hardware no hay EKF ni SLAM publicando TFs.
    # Cadena completa: map → odom → base_footprint → base_link → laser_link
    sim_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sim_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(sim_mode),
    )
    sim_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sim_odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        condition=IfCondition(sim_mode),
    )

    # ── 5. SLAM Toolbox (solo con hardware o si se pide explicitamente) ────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_yaml,
            'use_sim_time':     use_sim_time,
        }.items(),
        # En sim_mode SLAM no tiene datos de LIDAR; lo saltamos.
        # En hardware, slam:=true lo activa.
        condition=IfCondition(slam),
    )

    # ── 6. Nav2 ────────────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time':       use_sim_time,
            'params_file':        nav2_yaml,
            'use_lifecycle_mgr':  'true',
            'map':                map_path,
            'use_docking_server': 'false',
        }.items(),
    )

    # ── 7. twist_mux ──────────────────────────────────────────────────────────
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_yaml],
    )

    # ── 8. Foxglove Bridge ────────────────────────────────────────────────────
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'topic_whitelist': ['.*'],
            'max_qos_depth': 10,
            'num_threads': 2,
            'send_buffer_limit': 10000000,
            'use_compression': False,
        }],
    )

    return LaunchDescription([
        slam_arg,
        map_arg,
        use_camera_arg,
        sim_mode_arg,
        use_sim_time_arg,

        # Hardware (saltado en sim_mode)
        rrc_launch,
        ldlidar_launch,
        huskylens_launch,

        # TFs
        base_laser_tf,
        base_footprint_tf,
        sim_map_tf,
        sim_odom_tf,

        twist_mux_node,

        # EKF: esperar 2s (solo con hardware)
        TimerAction(period=2.0, actions=[ekf_node]),

        # SLAM: esperar 3s
        TimerAction(period=3.0, actions=[slam_launch]),

        # Nav2: esperar 5s
        TimerAction(period=5.0, actions=[nav2_launch]),

        foxglove_bridge,
    ])
