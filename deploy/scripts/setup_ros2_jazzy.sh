#!/bin/bash
# =============================================================================
# setup_ros2_jazzy.sh
# Instalación automática de ROS 2 Jazzy Jalisco en Ubuntu Server 24.04 LTS
# Para Raspberry Pi 4B — Proyecto Robot Autónomo (Robotica)
#
# Uso: chmod +x setup_ros2_jazzy.sh && ./setup_ros2_jazzy.sh
# =============================================================================

set -e  # Salir si hay algún error

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
log_ok()      { echo -e "${GREEN}[OK]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }
log_section() { echo -e "\n${BLUE}════════════════════════════════════════${NC}"; echo -e "${BLUE} $1${NC}"; echo -e "${BLUE}════════════════════════════════════════${NC}"; }

# Verificar Ubuntu 24.04
if ! grep -q "Ubuntu 24.04" /etc/os-release 2>/dev/null; then
    log_warn "Este script está diseñado para Ubuntu 24.04 (noble). Continuando igualmente..."
fi

# Verificar que NO somos root
if [ "$EUID" -eq 0 ]; then
    log_error "No ejecutes este script como root. Usa tu usuario normal (se pedirá sudo cuando sea necesario)."
fi

log_section "PASO 1/8 — Configurar locale"

sudo apt-get update
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
log_ok "Locale configurado"

log_section "PASO 2/8 — Habilitar repositorio Universe"

sudo apt-get install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt-get update
log_ok "Repositorio Universe habilitado"

log_section "PASO 3/8 — Añadir repositorio oficial de ROS 2 Jazzy"

sudo apt-get update && sudo apt-get install -y curl gnupg

# Añadir la clave GPG de ROS 2
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Añadir el repositorio de ROS 2 Jazzy (noble = Ubuntu 24.04)
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu noble main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update
log_ok "Repositorio de ROS 2 Jazzy añadido (noble)"

log_section "PASO 4/8 — Instalar ROS 2 Jazzy Base"

# ros-jazzy-ros-base: sin GUI (headless). Incluye:
# - rclpy / rclcpp
# - std_msgs, geometry_msgs, sensor_msgs, nav_msgs...
# - tf2, rosbag2
sudo apt-get install -y \
    ros-jazzy-ros-base \
    ros-dev-tools

log_ok "ROS 2 Jazzy Base instalado"

log_section "PASO 5/8 — Instalar herramientas de desarrollo Python y C++"

sudo apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    python3-pytest \
    build-essential \
    cmake \
    git \
    wget \
    nano \
    htop \
    i2c-tools \
    python3-smbus \
    python3-serial \
    libserial-dev \
    libudev-dev \
    libgpiod-dev \
    python3-gpiod

log_ok "Herramientas de desarrollo instaladas"

log_section "PASO 6/8 — Instalar paquetes ROS 2 base para robótica móvil"

sudo apt-get install -y \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-joy \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-diagnostic-msgs \
    ros-jazzy-std-srvs \
    ros-jazzy-rclpy \
    ros-jazzy-rclcpp \
    ros-jazzy-ament-cmake \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime \
    ros-jazzy-image-transport \
    ros-jazzy-compressed-image-transport \
    ros-jazzy-camera-info-manager \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-msgs \
    ros-jazzy-lifecycle-msgs \
    ros-jazzy-topic-tools \
    ros-jazzy-serial-driver \
    ros-jazzy-async-web-server-cpp \
    ros-jazzy-angles

log_ok "Paquetes ROS 2 de robótica instalados"

log_section "PASO 7/8 — Inicializar rosdep"

sudo rosdep init 2>/dev/null || log_warn "rosdep ya estaba inicializado"
rosdep update
log_ok "rosdep inicializado"

log_section "PASO 8/8 — Configurar el entorno (.bashrc)"

BASHRC="$HOME/.bashrc"

if ! grep -q "source /opt/ros/jazzy/setup.bash" "$BASHRC"; then
    cat >> "$BASHRC" << 'EOF'

# ── ROS 2 Jazzy ───────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash

# Workspace del robot (se activa si existe)
if [ -f "$HOME/robotica_ws/install/setup.bash" ]; then
    source "$HOME/robotica_ws/install/setup.bash"
fi

export ROS_DOMAIN_ID=42          # Aísla tu red ROS del resto (0-232)
export ROS_LOCALHOST_ONLY=0      # Permite comunicación en red local

# Autocompletado de ROS 2
eval "$(register-python-argcomplete ros2)"
eval "$(register-python-argcomplete colcon)"

# Aliases útiles
alias cb='cd ~/robotica_ws && colcon build --symlink-install'
alias cs='cd ~/robotica_ws && source install/setup.bash'
alias cbs='cd ~/robotica_ws && colcon build --symlink-install && source install/setup.bash'
alias rt='ros2 topic list'
alias rn='ros2 node list'
alias rl='ros2 launch'
alias rr='ros2 run'
alias roslog='cat ~/.ros/log/latest/rosout.log'
# ──────────────────────────────────────────────────────────────────────────
EOF
    log_ok ".bashrc configurado con ROS 2 Jazzy"
else
    log_warn ".bashrc ya tenía la configuración de ROS 2 (no se ha modificado)"
fi

# Habilitar I2C, SPI y UART en la RPi4 con Ubuntu 24.04
log_section "Extra — Habilitar I2C, SPI y UART"

CONFIG="/boot/firmware/config.txt"
if [ -f "$CONFIG" ]; then
    # I2C
    if ! grep -q "dtparam=i2c_arm=on" "$CONFIG"; then
        echo "dtparam=i2c_arm=on"    | sudo tee -a "$CONFIG"
        log_ok "I2C habilitado"
    else
        log_warn "I2C ya estaba habilitado"
    fi

    # SPI
    if ! grep -q "dtparam=spi=on" "$CONFIG"; then
        echo "dtparam=spi=on"        | sudo tee -a "$CONFIG"
        log_ok "SPI habilitado"
    else
        log_warn "SPI ya estaba habilitado"
    fi

    # UART
    if ! grep -q "enable_uart=1" "$CONFIG"; then
        echo "enable_uart=1"         | sudo tee -a "$CONFIG"
        log_ok "UART habilitado"
    else
        log_warn "UART ya estaba habilitado"
    fi

    # I2C del GPU (necesario para algunos sensores)
    if ! grep -q "dtparam=i2c_vc=on" "$CONFIG"; then
        echo "dtparam=i2c_vc=on"     | sudo tee -a "$CONFIG"
        log_ok "I2C_VC habilitado"
    fi
else
    log_warn "No se encontró $CONFIG. ¿Estás en una RPi con Ubuntu 24.04?"
fi

# Añadir usuario a los grupos necesarios para acceso sin sudo
sudo usermod -aG dialout,i2c,spi,gpio "$USER"
log_ok "Usuario $USER añadido a grupos dialout, i2c, spi, gpio"

# =============================================================================
# Verificación final
# =============================================================================
log_section "VERIFICACIÓN — Comprobando instalación"

source /opt/ros/jazzy/setup.bash

if ros2 --version &>/dev/null; then
    log_ok "ROS 2 Jazzy instalado correctamente: $(ros2 --version)"
else
    log_error "Algo falló con la instalación de ROS 2. Revisa los logs."
fi

echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║         ✅  ROS 2 JAZZY INSTALADO CORRECTAMENTE          ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "⚠️  Necesitas reiniciar (o desconectar y reconectar SSH) para que"
echo -e "   los cambios de grupo (i2c, spi, gpio) y .bashrc tengan efecto."
echo ""
echo -e "Próximo paso: ejecuta ${YELLOW}./setup_workspace.sh${NC} para crear el workspace"
echo ""
