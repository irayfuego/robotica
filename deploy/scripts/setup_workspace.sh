#!/bin/bash
# =============================================================================
# setup_workspace.sh
# Crea y configura el workspace ROS 2 del robot autónomo
#
# Uso: chmod +x setup_workspace.sh && ./setup_workspace.sh
# Ejecutar DESPUÉS de setup_ros2_jazzy.sh
# =============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
log_ok()      { echo -e "${GREEN}[OK]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_section() { echo -e "\n${BLUE}════════════════════════════════════════${NC}"; echo -e "${BLUE} $1${NC}"; echo -e "${BLUE}════════════════════════════════════════${NC}"; }

source /opt/ros/jazzy/setup.bash

WS_DIR="$HOME/robotica_ws"

log_section "Creando workspace ROS 2 en $WS_DIR"

# Crear estructura de directorios del workspace
mkdir -p "$WS_DIR/src"

# ─── Paquete: robotica_bringup ─────────────────────────────────────────────
# Paquete de lanzamiento principal del robot
mkdir -p "$WS_DIR/src/robotica_bringup/launch"
mkdir -p "$WS_DIR/src/robotica_bringup/config"

cat > "$WS_DIR/src/robotica_bringup/package.xml" << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robotica_bringup</name>
  <version>0.1.0</version>
  <description>Paquete de lanzamiento principal del robot autónomo</description>
  <maintainer email="pi@robotica.local">Robot Autonomo</maintainer>
  <license>MIT</license>

  <exec_depend>ros2launch</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

cat > "$WS_DIR/src/robotica_bringup/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(robotica_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
EOF

cat > "$WS_DIR/src/robotica_bringup/launch/robot.launch.py" << 'EOF'
"""
Launch principal del robot autónomo.
Añade aquí los nodos según vayas incorporando módulos.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Aquí irán los nodos del robot ──────────────────────────────────
        # Node(package='robotica_lidar', executable='lidar_node', ...),
        # Node(package='robotica_motors', executable='motor_node', ...),
        # ...
    ])
EOF

log_ok "Paquete robotica_bringup creado"

# ─── Paquete: robotica_description ────────────────────────────────────────
# Descripción URDF/XACRO del robot
mkdir -p "$WS_DIR/src/robotica_description/urdf"
mkdir -p "$WS_DIR/src/robotica_description/launch"
mkdir -p "$WS_DIR/src/robotica_description/meshes"

cat > "$WS_DIR/src/robotica_description/package.xml" << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robotica_description</name>
  <version>0.1.0</version>
  <description>Descripción URDF del robot autónomo (geometría, sensores, articulaciones)</description>
  <maintainer email="pi@robotica.local">Robot Autonomo</maintainer>
  <license>MIT</license>

  <build_depend>xacro</build_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

cat > "$WS_DIR/src/robotica_description/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(robotica_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
EOF

cat > "$WS_DIR/src/robotica_description/urdf/robot.urdf.xacro" << 'EOF'
<?xml version="1.0"?>
<robot name="robotica" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ── Propiedades del robot ── -->
  <xacro:property name="wheel_radius"    value="0.033"/>  <!-- metros -->
  <xacro:property name="wheel_width"     value="0.020"/>
  <xacro:property name="base_length"     value="0.200"/>
  <xacro:property name="base_width"      value="0.150"/>
  <xacro:property name="base_height"     value="0.050"/>

  <!-- ── Base ── -->
  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.4 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.006" iyz="0" izz="0.009"/>
    </inertial>
  </link>

  <!-- ── IMU (posición central) ── -->
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
  </joint>
  <link name="imu_link"/>

  <!-- ── LIDAR (posición frontal) ── -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.08 0 0.04" rpy="0 0 0"/>
  </joint>
  <link name="lidar_link"/>

  <!-- ── Gimbal/Cámara (servo x2) ── -->
  <joint name="gimbal_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="gimbal_pan_link"/>
    <origin xyz="0.05 0 0.08" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="3.14"/>
  </joint>
  <link name="gimbal_pan_link"/>

  <joint name="gimbal_tilt_joint" type="revolute">
    <parent link="gimbal_pan_link"/>
    <child link="gimbal_tilt_link"/>
    <origin xyz="0 0 0.02" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.785" upper="0.785" effort="10" velocity="3.14"/>
  </joint>
  <link name="gimbal_tilt_link"/>

  <!-- ── Cámara HuskyLens en el gimbal ── -->
  <joint name="camera_joint" type="fixed">
    <parent link="gimbal_tilt_link"/>
    <child link="camera_link"/>
    <origin xyz="0.02 0 0" rpy="0 0 0"/>
  </joint>
  <link name="camera_link"/>

</robot>
EOF

log_ok "Paquete robotica_description creado con URDF base"

# ─── Paquete: robotica_msgs ───────────────────────────────────────────────
# Mensajes y servicios custom del robot
mkdir -p "$WS_DIR/src/robotica_msgs/msg"
mkdir -p "$WS_DIR/src/robotica_msgs/srv"

cat > "$WS_DIR/src/robotica_msgs/package.xml" << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robotica_msgs</name>
  <version>0.1.0</version>
  <description>Mensajes y servicios ROS 2 custom del robot autónomo</description>
  <maintainer email="pi@robotica.local">Robot Autonomo</maintainer>
  <license>MIT</license>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

cat > "$WS_DIR/src/robotica_msgs/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(robotica_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Mensajes custom (añade aquí los .msg que vayas creando)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EncoderData.msg"
  "msg/MotorCommand.msg"
  "msg/ServoCommand.msg"
  DEPENDENCIES std_msgs
)

ament_package()
EOF

cat > "$WS_DIR/src/robotica_msgs/msg/EncoderData.msg" << 'EOF'
# Datos de los encoders de los 4 motores TT
std_msgs/Header header
float32[4] position    # posición en radianes [FL, FR, RL, RR]
float32[4] velocity    # velocidad en rad/s [FL, FR, RL, RR]
int32[4]   ticks       # ticks de encoder raw [FL, FR, RL, RR]
EOF

cat > "$WS_DIR/src/robotica_msgs/msg/MotorCommand.msg" << 'EOF'
# Comando de velocidad para los 4 motores TT
# Valores en [-1.0, 1.0] (porcentaje de velocidad máxima)
float32 front_left
float32 front_right
float32 rear_left
float32 rear_right
EOF

cat > "$WS_DIR/src/robotica_msgs/msg/ServoCommand.msg" << 'EOF'
# Comando de posición para los servos del gimbal
# Valores en radianes
float32 pan    # servo horizontal (eje Z)  rango: -1.57 a 1.57
float32 tilt   # servo vertical (eje Y)    rango: -0.78 a 0.78
EOF

log_ok "Paquete robotica_msgs creado con mensajes custom"

# ─── Paquete: robotica_base ───────────────────────────────────────────────
# Controlador base del robot (diferencial + odometría)
mkdir -p "$WS_DIR/src/robotica_base/robotica_base"
mkdir -p "$WS_DIR/src/robotica_base/launch"
mkdir -p "$WS_DIR/src/robotica_base/config"

cat > "$WS_DIR/src/robotica_base/package.xml" << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robotica_base</name>
  <version>0.1.0</version>
  <description>Controlador base del robot: odometría y conversión cmd_vel → motores</description>
  <maintainer email="pi@robotica.local">Robot Autonomo</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>robotica_msgs</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

cat > "$WS_DIR/src/robotica_base/setup.py" << 'EOF'
from setuptools import setup
import os
from glob import glob

package_name = 'robotica_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Autonomo',
    maintainer_email='pi@robotica.local',
    description='Controlador base del robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_node = robotica_base.base_node:main',
            'odometry_node = robotica_base.odometry_node:main',
        ],
    },
)
EOF

mkdir -p "$WS_DIR/src/robotica_base/resource"
touch "$WS_DIR/src/robotica_base/resource/robotica_base"
touch "$WS_DIR/src/robotica_base/robotica_base/__init__.py"

cat > "$WS_DIR/src/robotica_base/robotica_base/base_node.py" << 'EOF'
#!/usr/bin/env python3
"""
Nodo base del robot autónomo.
Convierte cmd_vel (Twist) en comandos de motor para la placa RRC Lite.
Este nodo será completado cuando se instale el driver de la RRC Lite.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robotica_msgs.msg import MotorCommand


class BaseNode(Node):
    """Controlador diferencial de 4 ruedas con tracción independiente."""

    def __init__(self):
        super().__init__('base_node')

        # Parámetros del robot (ajusta a tu geometría real)
        self.declare_parameter('wheel_base', 0.15)      # metros entre ejes izq/der
        self.declare_parameter('wheel_radius', 0.033)   # metros
        self.declare_parameter('max_speed', 1.0)        # m/s máximo

        self.wheel_base   = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_speed    = self.get_parameter('max_speed').value

        # Suscriptor de velocidad deseada
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publicador de comandos de motor
        self.motor_pub = self.create_publisher(
            MotorCommand, 'motor_command', 10)

        self.get_logger().info('BaseNode iniciado. Escuchando /cmd_vel...')

    def cmd_vel_callback(self, msg: Twist):
        """Convierte Twist a comandos individuales de motor (modelo diferencial)."""
        v = msg.linear.x    # velocidad lineal (m/s)
        w = msg.angular.z   # velocidad angular (rad/s)

        # Cinemática diferencial
        v_left  = (v - w * self.wheel_base / 2.0) / self.max_speed
        v_right = (v + w * self.wheel_base / 2.0) / self.max_speed

        # Limitar a [-1, 1]
        v_left  = max(-1.0, min(1.0, v_left))
        v_right = max(-1.0, min(1.0, v_right))

        cmd = MotorCommand()
        cmd.front_left  = v_left
        cmd.rear_left   = v_left
        cmd.front_right = v_right
        cmd.rear_right  = v_right

        self.motor_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

cat > "$WS_DIR/src/robotica_base/robotica_base/odometry_node.py" << 'EOF'
#!/usr/bin/env python3
"""
Nodo de odometría del robot.
Integra los datos de encoder para publicar la posición estimada del robot.
Será completado cuando se instale el driver de la RRC Lite.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from robotica_msgs.msg import EncoderData
import math


class OdometryNode(Node):
    """Calcula y publica la odometría del robot a partir de los encoders."""

    def __init__(self):
        super().__init__('odometry_node')

        self.declare_parameter('wheel_base',   0.15)
        self.declare_parameter('wheel_radius', 0.033)

        self.wheel_base   = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # Estado de la odometría
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.last_ticks = None

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publicador de odometría
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Suscriptor de encoders
        self.encoder_sub = self.create_subscription(
            EncoderData, 'encoder_data', self.encoder_callback, 10)

        self.get_logger().info('OdometryNode iniciado.')

    def encoder_callback(self, msg: EncoderData):
        """Integra los ticks de encoder para calcular la posición."""
        if self.last_ticks is None:
            self.last_ticks = msg.ticks
            return

        # Diferencia de ticks (promedio izq y der)
        d_left  = (msg.ticks[0] - self.last_ticks[0] +
                   msg.ticks[2] - self.last_ticks[2]) / 2.0
        d_right = (msg.ticks[1] - self.last_ticks[1] +
                   msg.ticks[3] - self.last_ticks[3]) / 2.0
        self.last_ticks = msg.ticks

        # Distancia en metros (ajusta ticks_per_rev a tu encoder)
        ticks_per_rev = 30.0  # típico para motores TT con encoder
        circumference = 2 * math.pi * self.wheel_radius
        d_l = (d_left  / ticks_per_rev) * circumference
        d_r = (d_right / ticks_per_rev) * circumference

        d     = (d_l + d_r) / 2.0
        delta = (d_r - d_l) / self.wheel_base

        self.x   += d * math.cos(self.yaw + delta / 2.0)
        self.y   += d * math.sin(self.yaw + delta / 2.0)
        self.yaw += delta

        # Publicar TF odom → base_footprint
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # Publicar Odometry
        odom = Odometry()
        odom.header.stamp    = t.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = t.transform.rotation.z
        odom.pose.pose.orientation.w = t.transform.rotation.w
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

cat > "$WS_DIR/src/robotica_base/config/robot_params.yaml" << 'EOF'
# Parámetros físicos del robot autónomo
# Ajusta estos valores a tu robot real

base_node:
  ros__parameters:
    wheel_base:   0.15     # metros entre ruedas izquierda y derecha
    wheel_radius: 0.033    # metros de radio de las ruedas TT
    max_speed:    0.5      # m/s velocidad máxima

odometry_node:
  ros__parameters:
    wheel_base:   0.15
    wheel_radius: 0.033
EOF

log_ok "Paquete robotica_base creado con nodos de control y odometría"

# ─── Compilar el workspace ─────────────────────────────────────────────────
log_section "Compilando el workspace ROS 2"

cd "$WS_DIR"
colcon build --symlink-install
log_ok "Workspace compilado correctamente"

# ─── Activar el workspace en .bashrc ──────────────────────────────────────
BASHRC="$HOME/.bashrc"
if ! grep -q "robotica_ws/install/setup.bash" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# Workspace Robotica" >> "$BASHRC"
    echo "source $WS_DIR/install/setup.bash" >> "$BASHRC"
    log_ok "Workspace añadido a .bashrc"
fi

source "$WS_DIR/install/setup.bash"

# ─── Resumen final ────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║        ✅  WORKSPACE DEL ROBOT CONFIGURADO               ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "Workspace creado en: ${YELLOW}$WS_DIR${NC}"
echo ""
echo "Paquetes en el workspace:"
echo "  📦 robotica_bringup     — launch principal del robot"
echo "  📦 robotica_description — URDF/XACRO del robot"
echo "  📦 robotica_msgs        — mensajes custom (EncoderData, MotorCommand, ServoCommand)"
echo "  📦 robotica_base        — controlador diferencial + odometría"
echo ""
echo -e "Para verificar que ROS 2 funciona: ${YELLOW}ros2 topic list${NC}"
echo -e "Para compilar el workspace:        ${YELLOW}cb${NC}  (alias)"
echo -e "Para lanzar el robot:              ${YELLOW}rl robotica_bringup robot.launch.py${NC}"
echo ""
echo -e "🎯 Siguiente paso: avisa cuando esté listo y se instalan los"
echo -e "   drivers de LIDAR D6, RRC Lite, GC9A01, HuskyLens 2 y TTP223"
echo ""
