#!/bin/bash
# ── robot_test.sh ─────────────────────────────────────────────────────────────
# Arranque en MODO TEST: solo el LIDAR + TFs estaticos + foxglove_bridge.
# Para usar mientras la placa RRC Lite (controladora de motores / odometria)
# NO esta conectada. El bringup completo (robot_start.sh) falla sin ella.
#
# Invocado por systemd via drop-in: robot-bringup.service.d/test-mode.conf
# Cuando llegue la RRC: borrar ese drop-in y daemon-reload para volver al
# bringup normal (robot_start.sh).
#
# Logs: journalctl -u robot-bringup -f   y   /tmp/robot_bringup.log

set -e

# Esperar a que el USB del LIDAR (CH340 -> /dev/sc_mini) este disponible
sleep 5

# Entorno ROS
source /opt/ros/jazzy/setup.bash
source /home/mimavi/robotica_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Log con timestamp
exec >> /tmp/robot_bringup.log 2>&1
echo "========================================"
echo "$(date): Arrancando robot en MODO TEST (solo LIDAR, sin RRC)..."
echo "========================================"

# Lanzar solo el LIDAR + TFs falsos + foxglove
exec ros2 launch robotica_bringup lidar_test.launch.py
