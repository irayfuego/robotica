#!/usr/bin/env python3
# encoding: utf-8
"""
hiwonder_board.py
SDK para la placa controladora Hiwonder (STM32F407VET6) — RRC Lite y variantes.

Protocolo serie:
    TX/RX: 0xAA 0x55 [Función] [Longitud] [Datos...] [CRC8]
    Baudios: 1 000 000, /dev/ttyACM0

Basado en el SDK original de Hiwonder para el MentorPi
(https://github.com/Matzefritz/HiWonder_MentorPi) — Licencia MIT.
Adaptado para el proyecto Robotica por VY.
"""

import enum
import time
import queue
import struct
import serial
import threading


# ---------------------------------------------------------------------------
# Tabla CRC-8
# ---------------------------------------------------------------------------
_CRC8_TABLE = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53,
]


def _crc8(data: bytes) -> int:
    check = 0
    for b in data:
        check = _CRC8_TABLE[check ^ b]
    return check & 0xFF


# ---------------------------------------------------------------------------
# Enumeraciones del protocolo
# ---------------------------------------------------------------------------
class _State(enum.IntEnum):
    START1   = 0
    START2   = 1
    FUNCTION = 2
    LENGTH   = 3
    DATA     = 4
    CHECKSUM = 5


class Func(enum.IntEnum):
    SYS       = 0
    LED       = 1
    BUZZER    = 2
    MOTOR     = 3
    PWM_SERVO = 4
    BUS_SERVO = 5
    KEY       = 6
    IMU       = 7
    GAMEPAD   = 8
    SBUS      = 9
    OLED      = 10
    RGB       = 11
    NONE      = 12


# ---------------------------------------------------------------------------
# Clase principal del board
# ---------------------------------------------------------------------------
class HiwonderBoard:
    """
    Interfaz Python para la placa controladora Hiwonder (RRC Lite / MentorPi).

    Uso básico:
        board = HiwonderBoard()
        board.enable_reception()
        board.set_motor_speed([[1, 0.5], [2, 0.5], [3, -0.5], [4, -0.5]])
        imu = board.get_imu()   # (ax, ay, az, gx, gy, gz)
    """

    def __init__(self, device: str = '/dev/ttyACM0', baudrate: int = 1_000_000,
                 timeout: float = 10.0):
        self._enable_recv = False
        self._frame: list = []
        self._state = _State.START1
        self._recv_count = 0

        self._port = serial.Serial(None, baudrate, timeout=timeout)
        self._port.rts = False
        self._port.dtr = False
        self._port.setPort(device)
        self._port.open()

        self._lock_pwm  = threading.Lock()
        self._lock_bus  = threading.Lock()

        self._q_sys      = queue.Queue(maxsize=1)
        self._q_imu      = queue.Queue(maxsize=1)
        self._q_key      = queue.Queue(maxsize=1)
        self._q_gamepad  = queue.Queue(maxsize=1)
        self._q_sbus     = queue.Queue(maxsize=1)
        self._q_bus_srv  = queue.Queue(maxsize=1)
        self._q_pwm_srv  = queue.Queue(maxsize=1)

        self._parsers = {
            Func.SYS:       self._on_sys,
            Func.IMU:       self._on_imu,
            Func.KEY:       self._on_key,
            Func.GAMEPAD:   self._on_gamepad,
            Func.SBUS:      self._on_sbus,
            Func.BUS_SERVO: self._on_bus_servo,
            Func.PWM_SERVO: self._on_pwm_servo,
        }

        time.sleep(0.5)
        threading.Thread(target=self._recv_loop, daemon=True).start()

    # ── Callbacks de recepción ──────────────────────────────────────────────
    def _put(self, q: queue.Queue, data):
        try:
            q.put_nowait(data)
        except queue.Full:
            pass

    def _on_sys(self, data):     self._put(self._q_sys,     data)
    def _on_imu(self, data):     self._put(self._q_imu,     data)
    def _on_key(self, data):     self._put(self._q_key,     data)
    def _on_gamepad(self, data): self._put(self._q_gamepad, data)
    def _on_sbus(self, data):    self._put(self._q_sbus,    data)
    def _on_bus_servo(self, data): self._put(self._q_bus_srv, data)
    def _on_pwm_servo(self, data): self._put(self._q_pwm_srv, data)

    # ── Envío ───────────────────────────────────────────────────────────────
    def _send(self, func: Func, data: list):
        buf = [0xAA, 0x55, int(func), len(data)] + data
        buf.append(_crc8(bytes(buf[2:])))
        self._port.write(bytes(buf))

    # ── Recepción (hilo daemon) ─────────────────────────────────────────────
    def _recv_loop(self):
        while True:
            if not self._enable_recv:
                time.sleep(0.01)
                continue
            raw = self._port.read()
            if not raw:
                continue
            for b in raw:
                if self._state == _State.START1:
                    if b == 0xAA:
                        self._state = _State.START2
                elif self._state == _State.START2:
                    self._state = _State.FUNCTION if b == 0x55 else _State.START1
                elif self._state == _State.FUNCTION:
                    if b < int(Func.NONE):
                        self._frame = [b, 0]
                        self._state = _State.LENGTH
                    else:
                        self._state = _State.START1
                elif self._state == _State.LENGTH:
                    self._frame[1] = b
                    self._recv_count = 0
                    self._state = _State.CHECKSUM if b == 0 else _State.DATA
                elif self._state == _State.DATA:
                    self._frame.append(b)
                    self._recv_count += 1
                    if self._recv_count >= self._frame[1]:
                        self._state = _State.CHECKSUM
                elif self._state == _State.CHECKSUM:
                    if _crc8(bytes(self._frame)) == b:
                        func = Func(self._frame[0])
                        payload = bytes(self._frame[2:])
                        if func in self._parsers:
                            self._parsers[func](payload)
                    self._state = _State.START1

    # ── API pública ─────────────────────────────────────────────────────────
    def enable_reception(self, enable: bool = True):
        """Activa/desactiva la recepción de datos del microcontrolador."""
        self._enable_recv = enable

    # Motores
    def set_motor_speed(self, speeds: list):
        """
        Establece la velocidad de los motores.
        speeds: lista de [motor_id (1-4), velocidad_rps (float)]
        Ejemplo: [[1, 1.5], [2, 1.5], [3, -1.5], [4, -1.5]]
        """
        data = [0x01, len(speeds)]
        for motor_id, rps in speeds:
            data += list(struct.pack('<Bf', int(motor_id) - 1, float(rps)))
        self._send(Func.MOTOR, data)

    def stop_motors(self):
        """Para todos los motores inmediatamente."""
        self.set_motor_speed([[1, 0.0], [2, 0.0], [3, 0.0], [4, 0.0]])

    # IMU
    def get_imu(self):
        """
        Devuelve (ax, ay, az, gx, gy, gz) o None si no hay datos.
        Aceleración en g, velocidad angular en °/s.
        """
        if not self._enable_recv:
            return None
        try:
            return struct.unpack('<6f', self._q_imu.get_nowait())
        except queue.Empty:
            return None

    # Servos PWM
    def pwm_servo_set_position(self, duration: float, positions: list):
        """
        Mueve los servos PWM a la posición indicada.
        duration: tiempo en segundos
        positions: lista de [servo_id (1-4), posicion (500-2500 µs)]
        """
        dur_ms = int(duration * 1000)
        data = [0x01, dur_ms & 0xFF, (dur_ms >> 8) & 0xFF, len(positions)]
        for srv_id, pos in positions:
            data += list(struct.pack('<BH', int(srv_id), int(pos)))
        self._send(Func.PWM_SERVO, data)

    def pwm_servo_set_offset(self, servo_id: int, offset: int):
        """Calibra el offset del servo PWM."""
        self._send(Func.PWM_SERVO, list(struct.pack('<BBb', 0x07, servo_id, int(offset))))

    # Buzzer y LED
    def set_buzzer(self, freq: int, on_time: float, off_time: float, repeat: int = 1):
        on_ms  = int(on_time  * 1000)
        off_ms = int(off_time * 1000)
        self._send(Func.BUZZER, list(struct.pack('<HHHH', freq, on_ms, off_ms, repeat)))

    def set_led(self, on_time: float, off_time: float, repeat: int = 1, led_id: int = 1):
        on_ms  = int(on_time  * 1000)
        off_ms = int(off_time * 1000)
        self._send(Func.LED, list(struct.pack('<BHHH', led_id, on_ms, off_ms, repeat)))

    # Batería
    def get_battery(self):
        """Devuelve tensión de batería en mV, o None si no hay datos."""
        if not self._enable_recv:
            return None
        try:
            data = self._q_sys.get_nowait()
            if data[0] == 0x04:
                return struct.unpack('<H', data[1:])[0]
        except queue.Empty:
            pass
        return None
