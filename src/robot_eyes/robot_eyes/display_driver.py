#!/usr/bin/env python3
"""
GC9A01 display driver for Raspberry Pi 4B.
Drives two round 240x240 displays via SPI (one per CS pin).

Wiring (default):
  Left eye  -> SPI0 CS0 (GPIO 8)
  Right eye -> SPI0 CS1 (GPIO 7)
  DC        -> GPIO 25
  RST       -> GPIO 27
  BL        -> GPIO 18 (PWM backlight)
  MOSI      -> GPIO 10
  SCLK      -> GPIO 11
"""

import time
import struct
import logging
from typing import Optional
from PIL import Image
import numpy as np

logger = logging.getLogger(__name__)

_GC9A01_SLPOUT     = 0x11
_GC9A01_INVON      = 0x21
_GC9A01_DISPON     = 0x29
_GC9A01_CASET      = 0x2A
_GC9A01_RASET      = 0x2B
_GC9A01_RAMWR      = 0x2C
_GC9A01_MADCTL     = 0x36
_GC9A01_COLMOD     = 0x3A

DISPLAY_WIDTH  = 240
DISPLAY_HEIGHT = 240


class GC9A01:
    """Driver for a single GC9A01 240x240 round display."""

    def __init__(self, spi, dc_pin, rst_pin, cs_pin,
                 bl_pin=None, rotation=0, gpio=None):
        self._spi = spi
        self._dc = dc_pin
        self._rst = rst_pin
        self._cs = cs_pin
        self._bl = bl_pin
        self._rotation = rotation
        self._gpio = gpio

    def begin(self, skip_hw_reset: bool = False):
        """Initialize the display.
        skip_hw_reset: skip RST pulse when RST is shared with another display."""
        GPIO = self._gpio
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self._dc,  GPIO.OUT)
        GPIO.setup(self._rst, GPIO.OUT)
        if self._bl is not None:
            GPIO.setup(self._bl, GPIO.OUT)
            GPIO.output(self._bl, GPIO.HIGH)

        if not skip_hw_reset:
            GPIO.output(self._rst, GPIO.HIGH); time.sleep(0.05)
            GPIO.output(self._rst, GPIO.LOW);  time.sleep(0.15)
            GPIO.output(self._rst, GPIO.HIGH); time.sleep(0.15)

        self._init_sequence()
        logger.info("GC9A01 display initialized (CS=%d)", self._cs)

    def _command(self, cmd):
        self._gpio.output(self._dc, self._gpio.LOW)
        self._spi.writebytes([cmd])

    def _data(self, data):
        self._gpio.output(self._dc, self._gpio.HIGH)
        if isinstance(data, int):
            self._spi.writebytes([data])
        else:
            chunk = 4096
            mv = memoryview(bytes(data))
            for i in range(0, len(mv), chunk):
                self._spi.writebytes2(mv[i:i+chunk])

    def _init_sequence(self):
        """GC9A01 full initialization sequence."""
        cmds = [
            (0xEF,[]),(0xEB,[0x14]),(0xFE,[]),(0xEF,[]),(0xEB,[0x14]),
            (0x84,[0x40]),(0x85,[0xFF]),(0x86,[0xFF]),(0x87,[0xFF]),
            (0x88,[0x0A]),(0x89,[0x21]),(0x8A,[0x00]),(0x8B,[0x80]),
            (0x8C,[0x01]),(0x8D,[0x01]),(0x8E,[0xFF]),(0x8F,[0xFF]),
            (0xB6,[0x00,0x00]),
            (_GC9A01_MADCTL,[self._get_madctl()]),
            (_GC9A01_COLMOD,[0x05]),
            (0x90,[0x08,0x08,0x08,0x08]),(0xBD,[0x06]),(0xBC,[0x00]),
            (0xFF,[0x60,0x01,0x04]),(0xC3,[0x13]),(0xC4,[0x13]),(0xC9,[0x22]),
            (0xBE,[0x11]),(0xE1,[0x10,0x0E]),(0xDF,[0x21,0x0c,0x02]),
            (0xF0,[0x45,0x09,0x08,0x08,0x26,0x2A]),
            (0xF1,[0x43,0x70,0x72,0x36,0x37,0x6F]),
            (0xF2,[0x45,0x09,0x08,0x08,0x26,0x2A]),
            (0xF3,[0x43,0x70,0x72,0x36,0x37,0x6F]),
            (0xED,[0x1B,0x0B]),(0xAE,[0x77]),(0xCD,[0x63]),
            (0x70,[0x07,0x07,0x04,0x0E,0x0F,0x09,0x07,0x08,0x03]),(0xE8,[0x34]),
            (0x62,[0x18,0x0D,0x71,0xED,0x70,0x70,0x18,0x0F,0x71,0xEF,0x70,0x70]),
            (0x63,[0x18,0x11,0x71,0xF1,0x70,0x70,0x18,0x13,0x71,0xF3,0x70,0x70]),
            (0x64,[0x28,0x29,0xF1,0x01,0xF1,0x00,0x07]),
            (0x66,[0x3C,0x00,0xCD,0x67,0x45,0x45,0x10,0x00,0x00,0x00]),
            (0x67,[0x00,0x3C,0x00,0x00,0x00,0x01,0x54,0x10,0x32,0x98]),
            (0x74,[0x10,0x85,0x80,0x00,0x00,0x4E,0x00]),(0x98,[0x3e,0x07]),
            (_GC9A01_INVON,[]),(0x11,[]),
        ]
        for c, d in cmds:
            self._command(c)
            if d:
                self._data(d)
            time.sleep(0.002)
        time.sleep(0.12)
        self._command(_GC9A01_DISPON)
        time.sleep(0.02)

    def _get_madctl(self):
        # Rotacion por HARDWARE via MADCTL (se aplica una vez en el init).
        # Base 0x18 = ML + BGR. Para 180 grados se anaden los bits MY (0x80) y
        # MX (0x40) -> 0xD8, que invierte el orden de filas Y columnas (giro de
        # 180, no un simple espejo). Es la forma correcta: no recalcula el
        # frame en cada refresco como hacia el flip de numpy (que ademas no
        # pintaba en hardware).
        return 0xD8 if self._rotation == 180 else 0x18

    def set_window(self, x0, y0, x1, y1):
        self._command(_GC9A01_CASET)
        self._data([x0>>8, x0&0xFF, x1>>8, x1&0xFF])
        self._command(_GC9A01_RASET)
        self._data([y0>>8, y0&0xFF, y1>>8, y1&0xFF])
        self._command(_GC9A01_RAMWR)

    def display_image(self, image: Image.Image):
        """Send a PIL Image (RGB 240x240) to the display via RGB565."""
        if image.size != (DISPLAY_WIDTH, DISPLAY_HEIGHT):
            image = image.resize((DISPLAY_WIDTH, DISPLAY_HEIGHT))
        if image.mode != "RGB":
            image = image.convert("RGB")
        arr = np.array(image, dtype=np.uint8)
        r = arr[:,:,0].astype(np.uint16)
        g = arr[:,:,1].astype(np.uint16)
        b = arr[:,:,2].astype(np.uint16)
        rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        pixel_bytes = rgb565.byteswap().tobytes()
        self.set_window(0, 0, DISPLAY_WIDTH-1, DISPLAY_HEIGHT-1)
        self._data(pixel_bytes)

    def fill(self, color: tuple):
        """Fill display with a solid RGB color."""
        r, g, b = color
        c16 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        c_bytes = struct.pack(">H", c16) * (DISPLAY_WIDTH * DISPLAY_HEIGHT)
        self.set_window(0, 0, DISPLAY_WIDTH-1, DISPLAY_HEIGHT-1)
        self._data(list(c_bytes))

    def backlight(self, on: bool):
        if self._bl is not None:
            self._gpio.output(self._bl, self._gpio.HIGH if on else self._gpio.LOW)

    def cleanup(self):
        self.backlight(False)


class DualDisplayController:
    """Manages two GC9A01 displays (left eye, right eye)."""

    DEFAULT_CONFIG = {
        "spi_bus":   0,
        "dc_pin":    25,
        "rst_pin":   27,
        "bl_pin":    18,
        "cs_left":   8,          # CE0
        "cs_right":  7,          # CE1
        "spi_speed": 40_000_000,
        "rotation":  0,          # 180 = pantallas montadas al reves
    }

    def __init__(self, config: Optional[dict] = None):
        self._cfg = {**self.DEFAULT_CONFIG, **(config or {})}
        self._left:  Optional[GC9A01] = None
        self._right: Optional[GC9A01] = None
        self._spi_left  = None
        self._spi_right = None
        self._gpio = None

    def begin(self):
        """Initialize both displays."""
        try:
            import RPi.GPIO as GPIO
            import spidev
        except ImportError as e:
            raise RuntimeError(f"Missing dependency: {e}\nInstall: pip install RPi.GPIO spidev") from e

        self._gpio = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        cfg = self._cfg

        # Open SPI devices
        self._spi_left = spidev.SpiDev()
        self._spi_left.open(cfg["spi_bus"], 0)
        self._spi_left.max_speed_hz = cfg["spi_speed"]
        self._spi_left.mode = 0

        self._spi_right = spidev.SpiDev()
        self._spi_right.open(cfg["spi_bus"], 1)
        self._spi_right.max_speed_hz = cfg["spi_speed"]
        self._spi_right.mode = 0

        # bl_pin < 0 (o ausente) = backlight NO controlado por GPIO (cableado a
        # VCC). Imprescindible para liberar GPIO18 cuando se usa para I2S.
        bl_pin = cfg["bl_pin"] if cfg["bl_pin"] is not None and cfg["bl_pin"] >= 0 else None

        self._left = GC9A01(
            spi=self._spi_left, dc_pin=cfg["dc_pin"], rst_pin=cfg["rst_pin"],
            cs_pin=cfg["cs_left"], bl_pin=bl_pin, gpio=GPIO,
            rotation=cfg["rotation"],
        )
        self._right = GC9A01(
            spi=self._spi_right, dc_pin=cfg["dc_pin"], rst_pin=cfg["rst_pin"],
            cs_pin=cfg["cs_right"], bl_pin=None, gpio=GPIO,
            rotation=cfg["rotation"],
        )

        # Hardware reset ONCE — RST is shared between both displays.
        # If we called begin() on each display separately, the second begin()
        # would reset the first display's initialized state, leaving it blank.
        GPIO.setup(cfg["dc_pin"],  GPIO.OUT)
        GPIO.setup(cfg["rst_pin"], GPIO.OUT)
        if bl_pin is not None:
            GPIO.setup(bl_pin, GPIO.OUT)
            GPIO.output(bl_pin, GPIO.HIGH)
        GPIO.output(cfg["rst_pin"], GPIO.HIGH); time.sleep(0.05)
        GPIO.output(cfg["rst_pin"], GPIO.LOW);  time.sleep(0.15)
        GPIO.output(cfg["rst_pin"], GPIO.HIGH); time.sleep(0.15)

        # Init each display (no extra RST pulse)
        self._left.begin(skip_hw_reset=True)
        self._right.begin(skip_hw_reset=True)
        logger.info("Both displays initialized.")

    def update(self, left_image: Image.Image, right_image: Image.Image):
        """Push new frames to both displays."""
        self._left.display_image(left_image)
        self._right.display_image(right_image)

    def fill_both(self, color=(0, 0, 0)):
        self._left.fill(color)
        self._right.fill(color)

    def cleanup(self):
        if self._left:
            self._left.cleanup()
        if self._spi_left:
            self._spi_left.close()
        if self._spi_right:
            self._spi_right.close()
        if self._gpio:
            self._gpio.cleanup()
        logger.info("Display controller cleaned up.")
