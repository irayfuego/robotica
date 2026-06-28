# Robótica — Robot autónomo con ojos animados, voz/LLM y navegación

Robot doméstico tipo "mascota" sobre base **mecanum 4WD** controlada por ROS 2.
Combina:

- **Cara animada** en dos pantallas (ojos) con expresiones y seguimiento de la mirada.
- **Voz natural**: comandos de voz offline (Vosk) + razonamiento y visión con un **LLM (Google Gemini)**.
- **Navegación**: LIDAR 2D + **SLAM Toolbox** (mapeo) y **Nav2** (navegación autónoma).
- **Cámara HuskyLens 2** para detección/seguimiento de caras y objetos.

Todo corre en una **Raspberry Pi 4B** con **ROS 2 Jazzy** (Ubuntu 24.04).

> Proyecto personal de un solo desarrollador. El código se desarrolla y se ejecuta
> directamente en la Raspberry; este repo es la copia versionada del workspace.

---

## 1. Hardware

| Componente | Detalle |
|---|---|
| **Computadora** | Raspberry Pi 4B (2 GB RAM, sin swap por defecto) |
| **Alimentación Pi** | Powerbank USB 20000 mAh |
| **Controladora motriz** | **RRC Lite** (Hiwonder, STM32F407VET6) — `/dev/ttyACM0`, 1 Mbaud, protocolo `0xAA 0x55 [Func][Len][Data][CRC8]` |
| **Batería motores** | LiPo 7,4 V 6700 mAh (a la RRC) |
| **Chasis** | Mecanum 4WD. Motores NULLLAB TT metálicos, encoder Hall 12 PPR, reducción 1:90 |
| **IMU** | QMI8658 (integrada en la RRC Lite) |
| **LIDAR** | LIDAR 2D 360° por USB-serie — `/dev/ttyUSB0` (symlink udev `/dev/sc_mini`). Driver activo: **`m1ct_d2`** |
| **Cámara** | HuskyLens 2 (I2C + su propio MCP HTTP/SSE) |
| **Pantallas** | 2 displays redondos = los ojos |
| **Audio** | Micrófono INMP441 + amplificador MAX98357A por I2S (overlay googlevoicehat); altavoz. (También se puede usar el micro/altavoz de la HuskyLens) |
| **Mando** | 8BitDo Ultimate 2C Wireless por **dongle 2.4 GHz** (modo XInput) |

**Montaje a tener en cuenta (calibrado):**
- El chasis quedó montado **girado 180°** (la trasera es el frente) → la cinemática
  invierte `vx`/`vy` (`body_reversed=True`).
- El **LIDAR está montado girado 90°** → TF `base_link → laser_link` con `yaw = -1.5708`.
- El firmware de la RRC entrega ~22 % de la velocidad comandada → factor
  `rps_calib = 4.5` (calibrado midiendo en el suelo).

---

## 2. Software

- **ROS 2 Jazzy** sobre Ubuntu 24.04 (aarch64).
- **Navegación:** `slam_toolbox` (mapeo online async), `nav2` (Nav2), `robot_localization` (EKF), `twist_mux`.
- **Teleop:** `joy` + `teleop_twist_joy`.
- **Visualización:** `foxglove_bridge` (puerto 8765) → Foxglove Studio.
- **Voz/IA:** Vosk (STT offline en el Pi) + Google **Gemini** (`gemini-2.5-flash-lite`) para comandos en lenguaje natural y visión (¿qué ves?).

---

## 3. Estructura del workspace (`robotica_ws/src`)

| Paquete | Rol |
|---|---|
| **`robot_eyes`** | Cara/ojos animados, motor de expresiones, comandos de voz (Vosk + Gemini), TTS, y puente de mirada con la HuskyLens. Es lo que arranca `launch_eyes.py`. |
| **`rrc_lite_ros2`** | **Driver de la RRC Lite** (motores, odometría, IMU, batería, gimbal) + cinemática mecanum. Nodo activo de la base motriz. |
| **`m1ct_d2`** | **Driver del LIDAR 2D** (el que se usa). Publica `/scan`. |
| **`robotica_bringup`** | Launch files y configuración (SLAM, Nav2, EKF, twist_mux, teleop, TFs). El "pegamento" del sistema. |
| **`robotica_description`** | Descripción URDF (geometría, sensores, articulaciones). |
| **`robotica_msgs`** | Mensajes/servicios ROS 2 propios. |
| **`robotica_base`** | Controlador base genérico (odometría + cmd_vel→motores). La implementación en uso es `rrc_lite_ros2`. |
| **`huskylens2_ros2`** | Integración de la cámara HuskyLens 2. |
| **`ldlidar_stl_ros2`** | Driver alternativo para LIDAR LDROBOT LD06/LD19. **No se usa** (el LIDAR actual va con `m1ct_d2`); incluido como referencia. Repo original: `github.com/ldrobotSensorTeam/ldlidar_stl_ros2`. |

---

## 4. Arranque (systemd)

Dos servicios arrancan solos al encender el Pi:

| Servicio | Ejecuta | Qué levanta |
|---|---|---|
| `robot_eyes.service` | `/home/mimavi/launch_eyes.py` | Ojos + voz + mirada + TTS (un proceso, `MultiThreadedExecutor`) |
| `robot-bringup.service` | `/home/mimavi/robot_start.sh` | Bringup completo: `ros2 launch robotica_bringup robot.launch.py` |

El comportamiento de `robot-bringup` se puede cambiar con un **drop-in** en
`/etc/systemd/system/robot-bringup.service.d/` que sobreescriba el `ExecStart`
(p. ej. un modo "solo motores" o "solo LIDAR"). Sin drop-in = bringup completo.

```bash
# Logs
journalctl -u robot-bringup -f
journalctl -u robot_eyes  -f
tail -f /tmp/robot_bringup.log
```

---

## 5. Modos de uso (launch)

Todos viven en `robotica_bringup/launch/`:

| Launch | Para qué |
|---|---|
| **`robot.launch.py`** | Bringup **completo**: motores + LIDAR + EKF + SLAM + Nav2 + twist_mux + foxglove. (Pesado para el Pi 2GB — ver notas.) |
| **`mapping.launch.py`** | **Modo MAPEO** (ligero): motores + LIDAR + teleop + **EKF (IMU)** + SLAM, **sin Nav2**. Para crear el mapa conduciendo. |
| **`teleop.launch.py`** | Solo `joy_node` + `teleop_twist_joy` (mando). |
| **`lidar_test.launch.py`** | Solo el LIDAR + TFs, para verlo en Foxglove. |

### Conducción con el mando (8BitDo, modo XInput)
- **Mantén RB** pulsado = "hombre muerto" (sin él no se mueve).
- **Stick izquierdo**: adelante/atrás (+ lateral).
- **Stick derecho (horizontal)**: girar.
- **LB**: turbo.

---

## 6. Flujo de mapeo y navegación (arquitectura "Opción 2")

El Pi de 2 GB **no aguanta SLAM + Nav2 a la vez**, así que se separan:

### a) Mapear
```bash
sudo systemctl stop robot-bringup.service        # liberar puertos/CPU
ros2 launch robotica_bringup mapping.launch.py   # (o lanzado en background)
# conducir TODA la casa con el mando, despacio, volviendo al inicio (cierre de bucle)
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/casa
```
En Foxglove: `Display frame = map` y mira el tópico `/map`.

### b) Navegar (pendiente de montar)
Modo navegación ligero con **AMCL + map_server + Nav2** sobre el mapa guardado
(sin SLAM en vivo). Aún por implementar.

---

## 7. Cinemática, calibración y TFs

- **Cinemática mecanum** en `rrc_lite_ros2/mecanum_kinematics.py`:
  - `body_reversed=True` → invierte `vx`,`vy` (montaje 180°).
  - `rps_calib=4.5` → compensa el factor de velocidad del firmware.
- **Parámetros** en `rrc_lite_ros2/config/rrc_lite_params.yaml`
  (`max_linear_vel=0.25`, `max_angular_vel=1.5` bajos a propósito para no saturar
  con el factor ×4.5; `publish_odom_tf=false` → la TF `odom→base` la publica el EKF).
- **Odometría:** la del nodo es a lazo abierto (velocidad comandada). El **EKF**
  (`robotica_bringup/config/ekf.yaml`) fusiona `/odom_raw` (vx,vy) + el
  **giroscopio** de la IMU (`vyaw`) → giros fiables. Solo se fusiona `vyaw` de la
  IMU (la aceleración lineal hace diverger la posición).
- **TFs estáticas:** `base_footprint→base_link`, `base_link→laser_link`
  (`yaw=-1.5708`, LIDAR a 90°), `base_link→imu_link` (identidad).

---

## 8. Despliegue y compilación

```bash
cd ~/robotica_ws
colcon build --packages-select <paquete>
source install/setup.bash
```

⚠️ **Compilar C++ (rclcpp) en el Pi 2GB sin swap lo CONGELA por OOM.** Antes de un
build C++ (p. ej. `m1ct_d2`): parar el bringup, **añadir swap** y compilar en serie:
```bash
sudo systemctl stop robot-bringup.service
sudo fallocate -l 2G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile
MAKEFLAGS=-j1 colcon build --packages-select m1ct_d2 --parallel-workers 1 --executor sequential
```
Los paquetes Python (la mayoría) no tienen este problema.

---

## 9. Notas y gotchas conocidos

- **Pi 2GB:** SLAM + Nav2 + foxglove a la vez lo saturan (Nav2 se autodestruye por
  heartbeat). De ahí el modo mapeo separado.
- **QoS de `/scan`:** se publica en **best_effort** (SensorDataQoS). Suscriptores en
  *reliable* (panel de mensajes de Foxglove, rclpy por defecto) no reciben nada;
  usar `qos_profile_sensor_data`.
- **`static_transform_publisher`:** su nombre de proceso se trunca a `static_transfor`
  (15 chars); ojo al matar/relanzar para no acumular publicadores de TF en conflicto.
- **ROS_DOMAIN_ID=0** y `ROS_LOCALHOST_ONLY=0` en los servicios.

---

## 10. Estado del proyecto

**Funciona:** ojos + voz/LLM + visión; base motriz calibrada (cmd_vel mueve el robot);
LIDAR corregido; **mapeo con SLAM + EKF (giroscopio)** produce mapas correctos.

**Pendiente:** mapear la casa completa y guardar el mapa; montar el **modo navegación
autónoma** (AMCL + Nav2) y validar un goal; afinar el bias del giroscopio si deriva en
recorridos largos.
