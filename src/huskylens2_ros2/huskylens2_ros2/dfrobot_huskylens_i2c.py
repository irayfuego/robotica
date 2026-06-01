#!/usr/bin/env python3
"""
dfrobot_huskylens_i2c.py
Driver I2C mínimo para la HuskyLens 2 (DFRobot SEN0638).
Basado en la librería oficial DFRobot_HuskylensV2 (MIT License).
https://github.com/DFRobot/DFRobot_HuskylensV2

Protocolo I2C:
    Dirección por defecto: 0x32
    Comandos UART/I2C: [0x55][0xAA][0x11][LEN][CMD][...DATA...][CHECKSUM]
"""

import struct
import time

try:
    import smbus2 as smbus
    _SMBUS2 = True
except ImportError:
    import smbus
    _SMBUS2 = False


# ── Constantes de protocolo ──────────────────────────────────────────────────
_I2C_ADDR    = 0x32
_FRAME_HEAD  = [0x55, 0xAA, 0x11]

# Comandos
CMD_REQUEST_BLOCKS          = 0x20
CMD_REQUEST_ARROWS          = 0x21
CMD_REQUEST_LEARNED         = 0x23
CMD_REQUEST_BLOCKS_LEARNED  = 0x24
CMD_REQUEST_ARROWS_LEARNED  = 0x25
CMD_IS_PRO_SUPPORT          = 0x29
CMD_RETURN_INFO             = 0x29
CMD_RETURN_BLOCK            = 0x2A
CMD_RETURN_ARROW            = 0x2B
CMD_REQUEST_ALGO            = 0x2D
CMD_RETURN_OK               = 0x2E
CMD_RETURN_BUSY             = 0x2F
CMD_RETURN_NEED_PRO         = 0x3C
CMD_REQUEST_KNOCK           = 0x2C
CMD_REQUEST_FORGET          = 0x01
CMD_REQUEST_SAVE            = 0x32
CMD_REQUEST_LOAD            = 0x33
CMD_REQUEST_SCREENSHOT      = 0x0A

# Algoritmos disponibles
ALGO_FACE_RECOGNITION      = 0x01
ALGO_OBJECT_TRACKING       = 0x02
ALGO_OBJECT_RECOGNITION    = 0x03
ALGO_LINE_TRACKING         = 0x04
ALGO_COLOR_RECOGNITION     = 0x05
ALGO_TAG_RECOGNITION       = 0x06
ALGO_OBJECT_CLASSIFICATION = 0x07
ALGO_GESTURE_RECOGNITION   = 0x08
ALGO_POSE_RECOGNITION      = 0x09
ALGO_HAND_RECOGNITION      = 0x0A
ALGO_EMOTION_RECOGNITION   = 0x0B
ALGO_SEGMENT               = 0x0C
ALGO_OCR_RECOGNITION       = 0x0D
ALGO_LICENSE_RECOGNITION   = 0x0E
ALGO_QRCODE_RECOGNITION    = 0x0F
ALGO_BARCODE_RECOGNITION   = 0x10

ALGO_NAMES = {
    ALGO_FACE_RECOGNITION:      'face_recognition',
    ALGO_OBJECT_TRACKING:       'object_tracking',
    ALGO_OBJECT_RECOGNITION:    'object_recognition',
    ALGO_LINE_TRACKING:         'line_tracking',
    ALGO_COLOR_RECOGNITION:     'color_recognition',
    ALGO_TAG_RECOGNITION:       'tag_recognition',
    ALGO_GESTURE_RECOGNITION:   'gesture_recognition',
    ALGO_POSE_RECOGNITION:      'pose_recognition',
    ALGO_HAND_RECOGNITION:      'hand_recognition',
    ALGO_OCR_RECOGNITION:       'ocr_recognition',
    ALGO_QRCODE_RECOGNITION:    'qrcode_recognition',
    ALGO_BARCODE_RECOGNITION:   'barcode_recognition',
}


def _checksum(data: list[int]) -> int:
    return sum(data) & 0xFF


def _build_cmd(cmd: int, data: list[int] = None) -> list[int]:
    if data is None:
        data = []
    payload = [cmd] + data
    frame = _FRAME_HEAD + [len(payload)] + payload
    frame.append(_checksum(frame))
    return frame


class HuskyLensResult:
    """Un objeto detectado (bloque o flecha)."""
    __slots__ = ('x_center', 'y_center', 'width', 'height',
                 'x_origin', 'y_origin', 'x_target', 'y_target',
                 'id', 'is_block')

    def __init__(self):
        self.x_center = self.y_center = 0
        self.width    = self.height   = 0
        self.x_origin = self.y_origin = 0
        self.x_target = self.y_target = 0
        self.id       = 0
        self.is_block = True

    def __repr__(self):
        if self.is_block:
            return (f'Block(id={self.id}, cx={self.x_center}, cy={self.y_center}, '
                    f'w={self.width}, h={self.height})')
        else:
            return (f'Arrow(id={self.id}, from=({self.x_origin},{self.y_origin}), '
                    f'to=({self.x_target},{self.y_target}))')


class HuskyLensI2C:
    """
    Interfaz I2C para HuskyLens 2.

    Uso:
        hl = HuskyLensI2C(bus=1, addr=0x32)
        if hl.begin():
            hl.set_algorithm(ALGO_FACE_RECOGNITION)
            results = hl.get_results()
    """

    def __init__(self, bus: int = 1, addr: int = _I2C_ADDR):
        self._addr = addr
        try:
            self._bus = smbus.SMBus(bus)
        except Exception as e:
            raise RuntimeError(f'No se pudo abrir I2C bus {bus}: {e}')

    def begin(self, retries: int = 5) -> bool:
        """Verifica comunicación con la cámara. Devuelve True si OK."""
        for _ in range(retries):
            try:
                if self._knock():
                    return True
            except Exception:
                pass
            time.sleep(0.1)
        return False

    def set_algorithm(self, algo: int) -> bool:
        """Cambia el algoritmo de visión activo."""
        cmd = _build_cmd(CMD_REQUEST_ALGO, [algo & 0xFF, 0x00])
        self._write(cmd)
        time.sleep(0.1)
        resp = self._read_response()
        return resp is not None and resp[0] == CMD_RETURN_OK

    def get_results(self) -> list:
        """Solicita y devuelve todos los resultados detectados."""
        cmd = _build_cmd(CMD_REQUEST_LEARNED)
        self._write(cmd)
        return self._parse_results()

    def get_blocks(self) -> list:
        """Solo bloques (cajas delimitadoras)."""
        cmd = _build_cmd(CMD_REQUEST_BLOCKS)
        self._write(cmd)
        return [r for r in self._parse_results() if r.is_block]

    def forget(self) -> bool:
        """Borra todos los objetos aprendidos."""
        cmd = _build_cmd(CMD_REQUEST_FORGET)
        self._write(cmd)
        resp = self._read_response()
        return resp is not None and resp[0] == CMD_RETURN_OK

    # ── Internos ─────────────────────────────────────────────────────────────
    def _knock(self) -> bool:
        cmd = _build_cmd(CMD_REQUEST_KNOCK)
        self._write(cmd)
        resp = self._read_response()
        return resp is not None and resp[0] == CMD_RETURN_OK

    def _write(self, data: list[int]):
        """Envía bytes al dispositivo I2C."""
        try:
            for i in range(0, len(data), 32):
                chunk = data[i:i+32]
                self._bus.write_i2c_block_data(self._addr, chunk[0], chunk[1:])
                time.sleep(0.002)
        except Exception as e:
            raise IOError(f'Error I2C write: {e}')

    def _read_response(self, timeout: float = 0.2) -> list | None:
        """Lee la respuesta del dispositivo."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                # Leer cabecera (5 bytes)
                header = self._bus.read_i2c_block_data(self._addr, 0x55, 5)
                if header[:3] != _FRAME_HEAD:
                    time.sleep(0.005)
                    continue
                length = header[3]
                if length == 0:
                    return [header[4]]
                # Leer datos + checksum
                rest = self._bus.read_i2c_block_data(self._addr, 0x00, length + 1)
                payload = [header[4]] + list(rest[:length])
                # verificar checksum
                full = list(header) + list(rest[:length])
                if rest[length] == _checksum(full[:-1]):
                    return payload
            except Exception:
                pass
            time.sleep(0.01)
        return None

    def _parse_results(self) -> list:
        """Parsea una respuesta con múltiples objetos detectados."""
        results = []
        resp = self._read_response()
        if resp is None or resp[0] not in (CMD_RETURN_INFO, CMD_RETURN_BLOCK, CMD_RETURN_ARROW):
            return results

        # Respuesta INFO: [CMD_RETURN_INFO][n_blocks][n_arrows][algo]
        if resp[0] == CMD_RETURN_INFO and len(resp) >= 4:
            n_blocks = resp[1]
            n_arrows = resp[2]

            for _ in range(n_blocks):
                block_resp = self._read_response()
                if block_resp and block_resp[0] == CMD_RETURN_BLOCK and len(block_resp) >= 11:
                    r = HuskyLensResult()
                    r.is_block = True
                    # Estructura: cmd(1) + x_center(2) + y_center(2) + width(2) + height(2) + id(2)
                    vals = struct.unpack_from('<5H', bytes(block_resp[1:11]))
                    r.x_center, r.y_center, r.width, r.height, r.id = vals
                    results.append(r)

            for _ in range(n_arrows):
                arrow_resp = self._read_response()
                if arrow_resp and arrow_resp[0] == CMD_RETURN_ARROW and len(arrow_resp) >= 9:
                    r = HuskyLensResult()
                    r.is_block = False
                    vals = struct.unpack_from('<4H', bytes(arrow_resp[1:9]))
                    r.x_origin, r.y_origin, r.x_target, r.y_target = vals
                    if len(arrow_resp) >= 11:
                        r.id = struct.unpack_from('<H', bytes(arrow_resp[9:11]))[0]
                    results.append(r)

        return results
