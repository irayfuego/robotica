#!/usr/bin/env python3
"""
Eye renderer for GC9A01 round displays.
Draws realistic cartoon eyes with smooth animations.
"""

import math
import numpy as np
from PIL import Image, ImageDraw
from dataclasses import dataclass, field
from typing import Tuple, Optional


DISPLAY_SIZE = 240  # GC9A01 is 240x240


@dataclass
class EyeState:
    """Represents the current visual state of one eye."""
    # Pupil position normalized [-1, 1] where (0,0) is center
    gaze_x: float = 0.0
    gaze_y: float = 0.0

    # Eyelid openness [0.0 = fully closed, 1.0 = fully open]
    upper_lid: float = 1.0
    lower_lid: float = 1.0  # how much the lower lid rises (0=normal, 1=closed)

    # Pupil dilation [0.5 = normal, 0 = tiny, 1 = huge]
    pupil_size: float = 0.5

    # Iris color (R, G, B)
    iris_color: Tuple[int, int, int] = field(default_factory=lambda: (60, 120, 200))

    # Squint (narrows eye vertically, independent of lid)
    squint: float = 0.0  # 0 = normal, 1 = fully squinted

    # Eyebrow raise [-1 = furrowed, 0 = neutral, 1 = raised]
    # (rendered as part of the eye circle boundary)
    eyebrow: float = 0.0


class EyeRenderer:
    """
    Renders a cartoon eye onto a PIL Image suitable for the GC9A01 display.
    All dimensions are in pixels for a 240x240 circular display.
    """

    def __init__(self, size: int = DISPLAY_SIZE, background_color=(10, 10, 15)):
        self.size = size
        self.cx = size // 2
        self.cy = size // 2
        self.background_color = background_color

        # Eye geometry constants
        self.eye_radius = int(size * 0.42)          # Outer white sclera radius
        self.iris_radius = int(size * 0.22)          # Iris radius
        self.pupil_radius_base = int(size * 0.13)    # Base pupil radius
        self.highlight_radius = int(size * 0.05)     # Corneal highlight

        # Max gaze offset in pixels
        self.max_gaze_offset = int(size * 0.12)

    def render(self, state: EyeState) -> Image.Image:
        """Render the eye and return a PIL Image (RGB, 240x240)."""
        img = Image.new("RGB", (self.size, self.size), self.background_color)
        draw = ImageDraw.Draw(img)

        # --- Draw circular mask boundary (display is round) ---
        # Fill everything outside the circle with background
        self._draw_circular_background(draw)

        # --- Sclera (white of the eye) ---
        self._draw_sclera(draw, state)

        # --- Iris ---
        iris_x, iris_y = self._get_iris_center(state)
        self._draw_iris(draw, state, iris_x, iris_y)

        # --- Pupil ---
        self._draw_pupil(draw, state, iris_x, iris_y)

        # --- Corneal highlight (makes it look alive) ---
        self._draw_highlight(draw, iris_x, iris_y)

        # --- Eyelids ---
        self._draw_eyelids(draw, state)

        # --- Outer shadow/rim ---
        self._draw_rim(draw)

        return img

    def _draw_circular_background(self, draw: ImageDraw.Draw):
        """Fill the display circle with background color."""
        draw.ellipse(
            [0, 0, self.size - 1, self.size - 1],
            fill=self.background_color
        )

    def _draw_sclera(self, draw: ImageDraw.Draw, state: EyeState):
        """Draw the white sclera ellipse."""
        r = self.eye_radius
        cx, cy = self.cx, self.cy

        # Vertical squish based on squint
        ry = int(r * (1.0 - state.squint * 0.3))

        draw.ellipse(
            [cx - r, cy - ry, cx + r, cy + ry],
            fill=(245, 245, 248),
            outline=(200, 200, 205),
            width=2
        )

        # Add subtle veins/texture using slightly off-white
        # (simple: a few radial gradient approximations)
        for angle_deg in [30, 90, 150, 210, 270, 330]:
            angle = math.radians(angle_deg)
            x1 = cx + int(r * 0.3 * math.cos(angle))
            y1 = cy + int(ry * 0.3 * math.sin(angle))
            x2 = cx + int(r * 0.75 * math.cos(angle))
            y2 = cy + int(ry * 0.75 * math.sin(angle))
            draw.line([x1, y1, x2, y2], fill=(225, 210, 210), width=1)

    def _get_iris_center(self, state: EyeState) -> Tuple[int, int]:
        """Compute iris center in pixels from normalized gaze."""
        # Clamp gaze to unit circle
        gx, gy = state.gaze_x, state.gaze_y
        mag = math.sqrt(gx * gx + gy * gy)
        if mag > 1.0:
            gx /= mag
            gy /= mag

        # Apply travel limit so iris stays inside sclera
        max_travel = self.eye_radius - self.iris_radius - 4
        ix = self.cx + int(gx * max_travel)
        iy = self.cy + int(gy * max_travel)
        return ix, iy

    def _draw_iris(self, draw: ImageDraw.Draw, state: EyeState, ix: int, iy: int):
        """Draw the colored iris with radial gradient approximation."""
        r = self.iris_radius
        base_r, base_g, base_b = state.iris_color

        # Draw concentric rings from outside in to fake a gradient
        steps = 8
        for i in range(steps, -1, -1):
            t = i / steps  # 1.0 at edge, 0.0 at center
            cr = int(r * (i + 1) / (steps + 1))
            # Darken toward edge
            factor = 0.5 + 0.5 * (1.0 - t)
            color = (
                min(255, int(base_r * factor)),
                min(255, int(base_g * factor)),
                min(255, int(base_b * factor)),
            )
            draw.ellipse(
                [ix - cr, iy - cr, ix + cr, iy + cr],
                fill=color
            )

        # Limbal ring (dark border around iris)
        draw.ellipse(
            [ix - r, iy - r, ix + r, iy + r],
            outline=(20, 20, 30),
            width=2
        )

    def _draw_pupil(self, draw: ImageDraw.Draw, state: EyeState, ix: int, iy: int):
        """Draw the pupil."""
        # Scale pupil size with dilation
        r = int(self.pupil_radius_base * (0.4 + state.pupil_size * 1.2))
        r = max(4, min(r, self.iris_radius - 2))
        draw.ellipse(
            [ix - r, iy - r, ix + r, iy + r],
            fill=(8, 8, 12)
        )

    def _draw_highlight(self, draw: ImageDraw.Draw, ix: int, iy: int):
        """Draw corneal specular highlight — makes the eye look alive."""
        r = self.highlight_radius
        hx = ix - int(self.iris_radius * 0.35)
        hy = iy - int(self.iris_radius * 0.35)
        draw.ellipse(
            [hx - r, hy - r, hx + r, hy + r],
            fill=(255, 255, 255)
        )
        # Smaller secondary highlight
        r2 = max(2, r // 2)
        hx2 = ix + int(self.iris_radius * 0.2)
        hy2 = iy - int(self.iris_radius * 0.15)
        draw.ellipse(
            [hx2 - r2, hy2 - r2, hx2 + r2, hy2 + r2],
            fill=(220, 230, 255)
        )

    def _draw_eyelids(self, draw: ImageDraw.Draw, state: EyeState):
        """
        Draw upper and lower eyelids as filled polygons that cover the sclera.
        upper_lid=1.0 → eye fully open (lid at top)
        upper_lid=0.0 → eye fully closed (lid at center)
        """
        cx, cy = self.cx, self.cy
        r = self.eye_radius + 20  # Slightly larger to ensure coverage
        half = self.size

        # Upper eyelid: polygon from top of display down to lid position
        # Lid position: at top of sclera when open, at center when closed
        open_top = cy - self.eye_radius - 5
        lid_y_upper = open_top + int((cy - open_top) * (1.0 - state.upper_lid))

        # Build a polygon that covers everything above the eyelid curve
        upper_poly = self._make_lid_polygon(
            cx, cy, r,
            lid_y=lid_y_upper,
            from_top=True
        )
        if upper_poly:
            draw.polygon(upper_poly, fill=self.background_color)

        # Eyelid edge (dark skin line)
        draw.line(
            [cx - self.eye_radius, lid_y_upper, cx + self.eye_radius, lid_y_upper],
            fill=(30, 25, 20), width=3
        )

        # Lower eyelid
        lower_open_y = cy + self.eye_radius + 5
        lid_y_lower = lower_open_y - int((lower_open_y - cy) * state.lower_lid * 0.6)

        lower_poly = self._make_lid_polygon(
            cx, cy, r,
            lid_y=lid_y_lower,
            from_top=False
        )
        if lower_poly:
            draw.polygon(lower_poly, fill=self.background_color)

        # Draw a thick curved eyelash line on upper lid
        eyelash_y = lid_y_upper
        eyelash_r = self.eye_radius
        draw.arc(
            [cx - eyelash_r, cy - eyelash_r, cx + eyelash_r, cy + eyelash_r],
            start=200, end=340,
            fill=(15, 10, 10), width=5
        )

    def _make_lid_polygon(self, cx, cy, r, lid_y, from_top):
        """
        Build a rectangular polygon that covers the lid area.
        Uses a flat horizontal cut for simplicity (looks clean on round display).
        """
        if from_top:
            return [
                (0, 0),
                (self.size, 0),
                (self.size, lid_y),
                (0, lid_y),
            ]
        else:
            return [
                (0, lid_y),
                (self.size, lid_y),
                (self.size, self.size),
                (0, self.size),
            ]

    def _draw_rim(self, draw: ImageDraw.Draw):
        """Draw a subtle dark rim around the circular display edge."""
        draw.ellipse(
            [2, 2, self.size - 3, self.size - 3],
            outline=(5, 5, 8),
            width=4
        )
