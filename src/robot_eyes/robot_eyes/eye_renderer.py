#!/usr/bin/env python3
"""
Eye renderer for GC9A01 round displays (240x240).
Draws realistic cartoon eyes that fill the full circular display.

Expressiveness channels beyond the basic lids/iris/pupil:
  lid_tilt        -1..+1  slope of the upper lid edge. Negative = inner corner
                          down (furrowed/angry "V"); positive = outer corner
                          down (sad droop). Mirrored per eye via render(mirror=).
  lower_lid_curve  0..1   lower lid rises as an upward arc instead of a flat
                          line -> smiling "^_^" eyes.
  eyebrow         -1..+1  vertical bias of the upper lid line (raised opens the
                          eye wider; furrowed pushes it down). The round display
                          has no room for a separate brow, so the lid acts as one.
  pupil_style     'round' | 'heart' | 'spiral'   special pupils (love, dizzy).
  overlay         '' | 'tear' | 'sweat' | 'zz'   animated decorations drawn on
                          top (sad tear, nervous sweat drop, sleeping Zz).
Overlay/pupil animations are time-based (the renderer uses the wall clock for
their phase), so they stay alive even when the eye state itself is static.
"""

import math
import time
from PIL import Image, ImageDraw
from dataclasses import dataclass, field
from typing import Tuple


DISPLAY_SIZE = 240  # GC9A01 is 240x240 pixels, circular


@dataclass
class EyeState:
    """Represents the current visual state of one eye."""
    gaze_x:          float = 0.0
    gaze_y:          float = 0.0
    upper_lid:       float = 1.0   # 1.0=fully open, 0.0=fully closed
    lower_lid:       float = 0.0   # 0.0=normal, 1.0=fully risen
    pupil_size:      float = 0.5   # 0.0=tiny, 1.0=huge
    iris_color:      Tuple[int, int, int] = field(default_factory=lambda: (60, 120, 200))
    squint:          float = 0.0   # 0.0=normal, 1.0=fully squinted
    eyebrow:         float = 0.0   # -1=furrowed, 0=neutral, 1=raised
    lid_tilt:        float = 0.0   # -1=inner corner down (angry), +1=outer down (sad)
    lower_lid_curve: float = 0.0   # 0=flat lower lid, 1=full smile arc ^_^
    pupil_style:     str   = 'round'   # 'round' | 'heart' | 'spiral'
    overlay:         str   = ''        # '' | 'tear' | 'sweat' | 'zz'


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

        # Lid tilt: vertical offset of each lid corner at |lid_tilt| = 1
        self.tilt_max = int(size * 0.09)     # 21 px

    def render(self, state: EyeState, mirror: bool = False) -> Image.Image:
        """Render one eye. mirror=True flips the lid tilt horizontally so the
        inner corners of a left/right eye pair face each other."""
        img  = Image.new("RGB", (self.size, self.size), self.background_color)
        draw = ImageDraw.Draw(img)
        now  = time.time()

        self._draw_sclera(draw, state)

        ix, iy = self._get_iris_center(state)
        self._draw_iris(draw, state, ix, iy)
        self._draw_pupil(draw, state, ix, iy, now)
        self._draw_highlight(draw, state, ix, iy)
        self._draw_eyelids(draw, state, mirror)
        self._draw_overlay(draw, state, mirror, now)
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

    def _draw_pupil(self, draw, state, ix, iy, now):
        r = int(self.pupil_radius_base * (0.4 + state.pupil_size * 1.2))
        r = max(4, min(r, self.iris_radius - 2))

        if state.pupil_style == 'heart':
            s = min(int(r * 1.25), self.iris_radius - 4)
            self._draw_heart(draw, ix, iy, s, fill=(195, 28, 70))
            return

        draw.ellipse([ix - r, iy - r, ix + r, iy + r], fill=(8, 8, 12))

        if state.pupil_style == 'spiral':
            # Rotating spiral inside the pupil (dizzy)
            steps = 28
            a0 = (now * 4.0) % (2.0 * math.pi)
            pts = []
            for i in range(steps):
                t   = i / (steps - 1)
                ang = a0 + t * 4.0 * math.pi
                rad = r * (0.12 + 0.80 * t)
                pts.append((ix + rad * math.cos(ang), iy + rad * math.sin(ang)))
            draw.line(pts, fill=(205, 205, 215), width=3)

    def _draw_heart(self, draw, cx, cy, s, fill):
        """Heart of half-width s centered around (cx, cy)."""
        r  = int(s * 0.52)
        oy = cy - int(s * 0.30)
        draw.ellipse([cx - s, oy - r, cx, oy + r], fill=fill)
        draw.ellipse([cx, oy - r, cx + s, oy + r], fill=fill)
        draw.polygon([(cx - int(s * 0.96), oy + int(r * 0.40)),
                      (cx + int(s * 0.96), oy + int(r * 0.40)),
                      (cx, cy + s)], fill=fill)

    def _draw_highlight(self, draw, state, ix, iy):
        r  = self.highlight_radius
        hx = ix - int(self.iris_radius * 0.32)
        hy = iy - int(self.iris_radius * 0.32)
        draw.ellipse([hx - r, hy - r, hx + r, hy + r], fill=(255, 255, 255))
        r2  = max(2, r // 2)
        hx2 = ix + int(self.iris_radius * 0.18)
        hy2 = iy - int(self.iris_radius * 0.14)
        draw.ellipse([hx2 - r2, hy2 - r2, hx2 + r2, hy2 + r2],
                     fill=(220, 230, 255))

    def _draw_eyelids(self, draw, state, mirror):
        cx, cy = self.cx, self.cy

        # --- Upper eyelid ---
        # Lids naturally follow the vertical gaze: looking down lowers the lid,
        # looking up opens it a touch wider.
        lid_open = state.upper_lid
        if state.gaze_y > 0:
            lid_open -= state.gaze_y * 0.18
        else:
            lid_open -= state.gaze_y * 0.06   # gaze_y<0 (up) -> opens wider
        lid_open = max(0.0, min(1.0, lid_open))

        sclera_top = cy - self.sclera_ry
        lid_y = sclera_top + int((cy - sclera_top) * (1.0 - lid_open))
        # Eyebrow biases the lid line: raised brow opens wider, furrowed lowers.
        lid_y -= int(state.eyebrow * self.size * 0.04)
        lid_y = max(sclera_top - 6, min(cy, lid_y))

        # Tilt: inner corner vs outer corner. With mirror=False the inner
        # corner is the RIGHT edge of the image; mirror=True flips it.
        tilt_px = int(state.lid_tilt * self.tilt_max)
        y_inner = lid_y - tilt_px
        y_outer = lid_y + tilt_px
        if mirror:
            y_left, y_right = y_inner, y_outer
        else:
            y_left, y_right = y_outer, y_inner

        draw.polygon(
            [(0, 0), (self.size, 0),
             (self.size, y_right + 2), (0, y_left + 2)],
            fill=self.background_color,
        )

        # Curved eyelash edge along the top of the sclera, only when open
        if lid_open > 0.05:
            arc_box = [cx - self.sclera_rx, cy - self.sclera_ry,
                       cx + self.sclera_rx, cy + self.sclera_ry]
            draw.arc(arc_box, start=200, end=340, fill=(20, 15, 12), width=6)

        # --- Lower eyelid ---
        # A curved (smiling) lid rises higher than a flat one so the ^_^ arc
        # actually cuts into the eye instead of hiding below the sclera edge.
        sclera_bot  = cy + self.sclera_ry
        curve  = state.lower_lid_curve
        rise_f = 0.55 + 0.45 * curve
        lid_y_lower = sclera_bot - int((sclera_bot - cy) * state.lower_lid * rise_f)
        if curve > 0.05:
            # Smiling arc ^_^ : a wide dark ellipse rising from below. The
            # narrower the ellipse, the stronger the arc; the corners of the
            # eye drop away naturally with it.
            er_y = int(self.size * 0.50)
            er_x = int(self.size * (0.95 - 0.33 * curve))
            ecy  = lid_y_lower + er_y
            draw.ellipse([cx - er_x, ecy - er_y, cx + er_x, ecy + er_y],
                         fill=self.background_color)
        else:
            draw.polygon(
                [(0, lid_y_lower), (self.size, lid_y_lower),
                 (self.size, self.size), (0, self.size)],
                fill=self.background_color,
            )

    def _draw_overlay(self, draw, state, mirror, now):
        """Animated decorations drawn over the lids (tear, sweat drop, Zz)."""
        if not state.overlay:
            return
        cx, cy = self.cx, self.cy
        outer = -1 if not mirror else 1   # x sign of the OUTER side

        if state.overlay == 'tear':
            # Droplet sliding down the outer lower edge, looping every 2.4 s
            phase = (now % 2.4) / 2.4
            tx = cx + outer * int(self.size * 0.30)
            ty = cy + int(self.size * 0.16) + int(phase * self.size * 0.17)
            w  = max(4, int(self.size * 0.030 * (1.0 - 0.35 * phase)))
            h  = int(w * 1.5)
            draw.polygon([(tx, ty - h), (tx - w, ty), (tx + w, ty)],
                         fill=(150, 200, 245))
            draw.ellipse([tx - w, ty - w // 2, tx + w, ty + int(w * 1.4)],
                         fill=(150, 200, 245))

        elif state.overlay == 'sweat':
            # Nervous sweat drop near the upper outer edge, slow slide down
            phase = (now % 3.0) / 3.0
            tx = cx + outer * int(self.size * 0.33)
            ty = cy - int(self.size * 0.26) + int(phase * self.size * 0.10)
            w  = max(5, int(self.size * 0.038))
            draw.polygon([(tx, ty - int(w * 1.7)), (tx - w, ty), (tx + w, ty)],
                         fill=(170, 215, 250))
            draw.ellipse([tx - w, ty - w // 2, tx + w, ty + int(w * 1.5)],
                         fill=(170, 215, 250))

        elif state.overlay == 'zz':
            # Floating Z z z rising toward the upper corner while sleeping
            base_x = cx + outer * int(self.size * 0.16)
            base_y = cy - int(self.size * 0.05)
            for i in range(3):
                phase = ((now * 0.45) + i * 0.33) % 1.0
                zx = base_x + outer * int(phase * self.size * 0.13) \
                    + int(math.sin((now + i) * 2.0) * 3)
                zy = base_y - int(phase * self.size * 0.26)
                zs = int(self.size * (0.045 + 0.030 * (1.0 - phase)) * (1.0 + i * 0.12))
                self._draw_z(draw, zx, zy, zs, fill=(200, 205, 228))

    @staticmethod
    def _draw_z(draw, x, y, s, fill):
        """A 'Z' glyph drawn with 3 strokes, top-left corner at (x, y)."""
        w = max(2, s // 5)
        draw.line([(x, y), (x + s, y)], fill=fill, width=w)
        draw.line([(x + s, y), (x, y + s)], fill=fill, width=w)
        draw.line([(x, y + s), (x + s, y + s)], fill=fill, width=w)

    def _draw_rim(self, draw):
        """Subtle dark ring to frame the circular display edge."""
        draw.ellipse(
            [3, 3, self.size - 4, self.size - 4],
            outline=(5, 5, 8),
            width=5,
        )
