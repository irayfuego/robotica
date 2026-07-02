#!/bin/bash
# ── robot_start.sh ────────────────────────────────────────────────────────────
# Arranca el bringup completo del robot Mecanum.
# Invocado por systemd: robot-bringup.service
# Logs en: journalctl -u robot-bringup -f
#          /tmp/robot_bringup.log

set -e

# Esperar a que el hardware USB esté disponible (CH340, RRC Lite)
sleep 5

# Entorno ROS
source /opt/ros/jazzy/setup.bash
source /home/mimavi/robotica_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Log con timestamp
exec >> /tmp/robot_bringup.log 2>&1
echo "========================================"
echo "$(date): Arrancando robot bringup..."
echo "========================================"

# Lanzar bringup completo (con hardware real)
# Quitar use_camera:=false si la HuskyLens está conectada
MODE=$(cat /home/mimavi/robot_mode 2>/dev/null || echo mapping)
echo "$(date): modo seleccionado: $MODE"
case "$MODE" in
  navigation) MAP=$(cat /home/mimavi/robot_map 2>/dev/null || echo /home/mimavi/maps/casa.yaml)
              exec ros2 launch robotica_bringup navigation.launch.py map:="$MAP" ;;
  full)       exec ros2 launch robotica_bringup robot.launch.py use_camera:=false ;;
  *)          exec ros2 launch robotica_bringup mapping.launch.py ;;
esac

