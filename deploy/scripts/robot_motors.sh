#!/bin/bash
# ── robot_motors.sh ───────────────────────────────────────────────────────────
# Bringup LIGERO: SOLO el control de motores (rrc_lite_node).
#
# SLAM y Nav2 quedan FUERA a proposito: saturan el Pi 4 (se vio load average 25)
# y sin el LIDAR conectado no tienen datos, asi que solo consumen CPU. Cuando el
# LIDAR este conectado y la alimentacion sea suficiente, volver al bringup
# completo (robot_start.sh) borrando el drop-in motors-mode.conf.
#
# Logs: journalctl -u robot-bringup -f   y   /tmp/robot_bringup.log

set -e

# Esperar a que el USB de la RRC Lite (CH343 -> /dev/ttyACM0) este disponible
sleep 5

# Entorno ROS
source /opt/ros/jazzy/setup.bash
source /home/mimavi/robotica_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Log con timestamp
exec >> /tmp/robot_bringup.log 2>&1
echo "========================================"
echo "$(date): Arrancando MODO MOTORES (solo rrc_lite_node, sin SLAM/Nav2)..."
echo "========================================"

# Solo el driver de motores (recibe /cmd_vel, publica /odom_raw, /imu, bateria)
exec ros2 launch rrc_lite_ros2 rrc_lite.launch.py
