# Despliegue / recuperación ante desastre

Cómo volver a montar el robot **desde cero** en una Raspberry Pi nueva (o si muere
la SD/SSD), partiendo solo de este repositorio de GitHub.

Todo lo que vive **fuera** del workspace ROS (scripts de arranque, servicios
systemd, regla udev, config de audio) está respaldado en la carpeta **`deploy/`**.
Lo que **no** se sube por seguridad: la **clave de la API de Gemini** y el **modelo
de Vosk** (se indican abajo dónde van).

> Suposiciones: usuario `mimavi`, workspace en `/home/mimavi/robotica_ws`.
> Si cambias usuario/ruta, ajusta los scripts de `deploy/scripts/` (tienen rutas
> absolutas `/home/mimavi/...`).

---

## 1. Sistema base

1. Flashear **Ubuntu Server 24.04 LTS (64-bit)** en la SD/SSD del Pi 4B.
2. Usuario `mimavi`, habilitar SSH, conectar a la red.
3. Editar `/boot/firmware/config.txt` con las opciones de **`deploy/boot-config-snippet.txt`**
   (interfaces I2C/SPI, USB host, pantallas KMS y **audio I2S googlevoicehat**) y **reiniciar**.

## 2. ROS 2 Jazzy + workspace

```bash
git clone https://github.com/irayfuego/robotica.git ~/robotica_ws
cd ~/robotica_ws

# a) Instala ROS 2 Jazzy + dependencias apt (instalador, seguro en sistema limpio)
./deploy/scripts/setup_ros2_jazzy.sh

# b) Compila el workspace con el codigo de ESTE repo
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y    # si aplica
colcon build --symlink-install
source install/setup.bash
```

> ⚠️ **NO ejecutes `setup_workspace.sh` sobre el repo clonado.** Fue el *bootstrap
> original*: GENERA los paquetes desde cero (heredocs embebidos) y SOBREESCRIBIRIA
> el codigo actual con versiones antiguas. Se conserva en `deploy/scripts/` solo
> como referencia historica del montaje inicial.

> **LIDAR:** el driver en uso es **`m1ct_d2`** (ya está en el repo). El paquete
> alternativo `ldlidar_stl_ros2` NO se incluye (gitignored, no se usa); si alguna
> vez hiciera falta: `git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2 src/ldlidar_stl_ros2`.

> ⚠️ **Compilar C++ en el Pi 2GB** (p.ej. `m1ct_d2`) requiere swap o congela el Pi
> por OOM. Ver `deploy/scripts/build_lidar.sh` y el README (sección 8).

## 3. Dispositivos (udev)

```bash
sudo cp deploy/udev/sc_mini.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```
Crea el symlink `/dev/sc_mini` para el LIDAR (CH340). La RRC Lite aparece como
`/dev/ttyACM0` (no necesita regla).

## 4. Scripts de arranque + servicios systemd

```bash
# Scripts de bringup en el HOME (los que invocan los servicios)
cp deploy/scripts/robot_start.sh deploy/scripts/robot_motors.sh \
   deploy/scripts/robot_test.sh deploy/scripts/build_lidar.sh ~/
chmod +x ~/robot_start.sh ~/robot_motors.sh ~/robot_test.sh ~/build_lidar.sh

# Symlink que espera el servicio robot_eyes
ln -sf ~/robotica_ws/src/robot_eyes/scripts/launch_eyes.py ~/launch_eyes.py

# Servicios systemd
sudo cp deploy/systemd/robot-bringup.service deploy/systemd/robot_eyes.service \
   /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robot_eyes.service robot-bringup.service
```

## 5. Secretos y modelos (NO están en git)

**Clave de Gemini** (comandos de voz por LLM):
```bash
sudo mkdir -p /etc/systemd/system/robot_eyes.service.d
sudo cp deploy/systemd/robot_eyes.service.d/gemini.conf.template \
        /etc/systemd/system/robot_eyes.service.d/gemini.conf
sudo nano /etc/systemd/system/robot_eyes.service.d/gemini.conf   # poner la clave real
sudo systemctl daemon-reload && sudo systemctl restart robot_eyes
```

**Modelo Vosk** (STT offline): descargar el modelo en español de
https://alphacephei.com/vosk/models y colocarlo donde lo busca `voice_command_node`
(ver la ruta/param en `robot_eyes`). No se versiona por tamaño.

**HuskyLens 2:** la cámara expone su propio MCP (HTTP+SSE, puerto 3000); ver memoria
del proyecto / `huskylens2_ros2`.

## 6. Comprobación

```bash
systemctl status robot_eyes robot-bringup
ls -l /dev/ttyACM0 /dev/sc_mini        # RRC Lite + LIDAR presentes
journalctl -u robot-bringup -f
```
Para mapear/navegar, ver el **README.md** (secciones 5 y 6).

---

## Contenido de `deploy/`

```
deploy/
├── scripts/          # copias canónicas de los .sh del HOME + instaladores
│   ├── setup_ros2_jazzy.sh   # instala ROS 2 Jazzy + apt (seguro)
│   ├── setup_workspace.sh    # BOOTSTRAP ORIGINAL (genera el ws desde cero); NO ejecutar sobre el repo
│   ├── robot_start.sh        # bringup completo (robot.launch.py)
│   ├── robot_motors.sh       # bringup ligero (solo motores)
│   ├── robot_test.sh         # bringup de prueba del LIDAR
│   └── build_lidar.sh        # compila el LIDAR (C++) con swap
├── systemd/
│   ├── robot-bringup.service
│   ├── robot_eyes.service
│   └── robot_eyes.service.d/gemini.conf.template   # plantilla SIN la clave
├── udev/
│   └── sc_mini.rules         # symlink /dev/sc_mini para el LIDAR
└── boot-config-snippet.txt   # opciones de /boot/firmware/config.txt (audio I2S, etc.)
```
