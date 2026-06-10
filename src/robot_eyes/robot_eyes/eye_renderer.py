#!/usr/bin/env python3
"""
Eye renderer for GC9A01 round displays (240x240).
Draws realistic cartoon eyes that fill the full circular display.
"""

import math
import numpy as np
from PIL import Image, ImageDraw
from dataclasses import dataclass, field
from typing import Tuple, Optional


DISPLAY_SIZE = 240  # GC9A01 is 240x240 pixels, circular


@dataclass
class EyeState:
    """Represents the current visual state of one eye."""
    gaze_x:     float = 0.0
    gaze_y:     float = 0.0
    upper_lid:  float = 1.0   # 1.0=fully open, 0.0=fully closed
    lower_lid:  float = 0.0   # 0.0=normal, 1.0=fully risen
    pupil_size: float = 0.5   # 0.0=tiny, 1.0=huge
    iris_color: Tuple[int, int, int] = field(default_factory=lambda: (60, 120, 200))
    squint:     float = 0.0   # 0.0=normal, 1.0=fully squinted
    eyebrow:    float = 0.0   # -1=furrowed, 0=neutral, 1=raised


class EyeRenderer:
    """
    Renders a cartoon eye onto a PIL Image for the GC9A01 circular display.
    The sclera fills the full display width; eyelids clip the top/bottom.
    """

    def __init__(self, size: int = DISPLAY_SIZE, background_color=(10, 10, 15)):
        self.size = size
        self.cx = size // 2   # 120
        self.cy = size // 2   # 120
        self.background_color = background_color

        # --- Sclera ellipse (fills the circular display) ---
        # rx fills ~95% of display radius horizontally (near edge-to-edge)
        # ry gives natural eye aspect ratio (~0.80 of rx)
        self.sclera_rx = int(size * 0.488)   # 117 px  (display circle radius = 120)
        self.sclera_ry = int(size * 0.388)   # 93 px

        # --- Iris / pupil ---
        self.iris_radius       = int(size * 0.272)   # 65 px
        self.pupil_radius_base = int(size * 0.157)   # 38 px
        self.highlight_radius  = int(size * 0.058)   # 14 px

        # Max gaze offset: iris can travel this far from center
        self.max_gaze_x = self.sclera_rx - self.iris_radius - 5
        self.max_gaze_y = self.sclera_ry - self.iris_radius - 5

    def render(self, state: EyeState) -> Image.Image:
        img  = Image.new("RGB", (self.size, self.size), self.background_color)
        draw = ImageDraw.Draw(img)

        self._draw_sclera(draw, state)

        ix, iy = self._get_iris_center(state)
        self._draw_iris(draw, state, ix, iy)
        self._draw_pupil(draw, state, ix, iy)
        self._draw_highlight(draw, ix, iy)
        self._draw_eyelids(draw, state)
        self._draw_rim(draw)

        return img

    def _draw_sclera(self, draw, state):
        rx = self.sclera_rx
        ry = int(self.sclera_ry * (1.0 - state.squint * 0.30))
        cx, cy = self.cx, self.cy

        draw.ellipse(
            [cx - rx, cy - ry, cx + rx, cy + ry],
            fill=(245, 245, 248),
            outline=(200, 200, 205),
            width=2,
        )

        # Subtle vein lines for realism
        for deg in [30, 90, 150, 210, 270, 330]:
            angle = math.radians(deg)
            x1 = cx + int(rx * 0.30 * math.cos(angle))
            y1 = cy + int(ry * 0.30 * math.sin(angle))
            x2 = cx + int(rx * 0.72 * math.cos(angle))
            y2 = cy + int(ry * 0.72 * math.sin(angle))
            draw.line([x1, y1, x2, y2], fill=(225, 210, 210), width=1)

    def _get_iris_center(self, state):
        gx, gy = state.gaze_x, state.gaze_y
        mag = math.sqrt(gx * gx + gy * gy)
        if mag > 1.0:
            gx /= mag; gy /= mag
        ix = self.cx + int(gx * self.max_gaze_x)
        iy = self.cy + int(gy * self.max_gaze_y)
        return ix, iy

    def _draw_iris(self, draw, state, ix, iy):
        r = self.iris_radius
        base_r, base_g, base_b = state.iris_color
        steps = 10
        for i in range(steps, -1, -1):
            cr = max(1, int(r * (i + 1) / (steps + 1)))
            t  = i / steps
            factor = 0.45 + 0.55 * (1.0 - t)
            color = (
                min(255, int(base_r * factor)),
                min(255, int(base_g * factor)),
                min(255, int(base_b * factor)),
            )
            draw.ellipse([ix - cr, iy - cr, ix + cr, iy + cr], fill=color)
        draw.ellipse([ix - r, iy - r, ix + r, iy + r],
                     outline=(20, 20, 30), width=2)

    def _draw_pupil(self, draw, state, ix, iy):
        r = int(self.pupil_radius_base * (0.4 + state.pupil_size * 1.2))
        r = max(4, min(r, self.iris_radius - 2))
        draw.ellipse([ix - r, iy - r, ix + r, iy + r], fill=(8, 8, 12))

    def _draw_highlight(self, draw, ix, iy):
        r  = self.highlight_radius
        hx = ix - int(self.iris_radius * 0.32)
        hy = iy - int(self.iris_radius * 0.32)
        draw.ellipse([hx - r, hy - r, hx + r, hy + r], fill=(255, 255, 255))
        r2  = max(2, r // 2)
        hx2 = ix + int(self.iris_radius * 0.18)
        hy2 = iy - int(self.iris_radius * 0.14)
        draw.ellipse([hx2 - r2, hy2 - r2, hx2 + r2, hy2 + r2],
                     fill=(220, 230, 255))

    def _draw_eyelids(self, draw, state):
        cx, cy = self.cx, self.cy

        # --- Upper eyelid ---
        # Fully open: lid sits at top of sclera  (cy - sclera_ry)
        # Fully closed: lid travels down to cy
        sclera_top = cy - self.sclera_ry
        lid_y_upper = sclera_top + int((cy - sclera_top) * (1.0 - state.upper_lid))

        # Cover everything above the lid with background
        draw.polygon(
            [(0, 0), (self.size, 0),
             (self.size, lid_y_upper), (0, lid_y_upper)],
            fill=self.background_color,
        )

        # Curved eyelid edge (arc following upper sclera curvature)
        # Draw as a thick dark line at the lid boundary
        arc_box = [cx - self.sclera_rx, cy - self.sclera_ry,
                   cx + self.sclera_rx, cy + self.sclera_ry]
        # The upper eyelid line is approximated by the sclera arc from ~200 to ~340 deg
        # but shifted down to match lid_y_upper when partially closed
        if state.upper_lid > 0.05:
            # Only draw eyelash when eye is at least slightly open
            draw.arc(arc_box, start=200, end=340,
                     fill=(20, 15, 12), width=6)

        # Eyelid skin shadow just above the lash line
        draw.polygon(
            [(0, 0), (self.size, 0),
             (self.size, lid_y_upper + 2), (0, lid_y_upper + 2)],
            fill=self.background_color,
        )

        # --- Lower eyelid ---
        sclera_bot = cy + self.sclera_ry
        lid_y_lower = sclera_bot - int((sclera_bot - cy) * state.lower_lid * 0.55)

        draw.polygon(
            [(0, lid_y_lower), (self.size, lid_y_lower),
             (self.size, self.size), (0, self.size)],
            fill=self.background_color,
        )

    def _draw_rim(self, draw):
        """Subtle dark ring to frame the circular display edge."""
        draw.ellipse(
            [3, 3, self.size - 4, self.size - 4],
            outline=(5, 5, 8),
            width=5,
        )
