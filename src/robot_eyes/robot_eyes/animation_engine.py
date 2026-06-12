#!/usr/bin/env python3
"""
Animation engine for robot eyes.
Handles smooth interpolation between states and behavior sequences.
"""

import time
import math
import random
import threading
from dataclasses import dataclass, field
from typing import List, Optional, Callable
from enum import Enum, auto

from .eye_renderer import EyeState


class EasingType(Enum):
    LINEAR      = auto()
    EASE_IN     = auto()
    EASE_OUT    = auto()
    EASE_IN_OUT = auto()
    ELASTIC     = auto()
    BOUNCE      = auto()


def ease(t: float, easing: EasingType) -> float:
    t = max(0.0, min(1.0, t))
    if easing == EasingType.LINEAR:
        return t
    elif easing == EasingType.EASE_IN:
        return t * t
    elif easing == EasingType.EASE_OUT:
        return 1.0 - (1.0 - t) ** 2
    elif easing == EasingType.EASE_IN_OUT:
        return t * t * (3.0 - 2.0 * t)
    elif easing == EasingType.ELASTIC:
        if t == 0.0 or t == 1.0:
            return t
        p = 0.3; s = p / 4.0
        return (2.0 ** (-10 * t)) * math.sin((t - s) * (2 * math.pi) / p) + 1.0
    elif easing == EasingType.BOUNCE:
        if t < 1 / 2.75:
            return 7.5625 * t * t
        elif t < 2 / 2.75:
            t -= 1.5 / 2.75; return 7.5625 * t * t + 0.75
        elif t < 2.5 / 2.75:
            t -= 2.25 / 2.75; return 7.5625 * t * t + 0.9375
        else:
            t -= 2.625 / 2.75; return 7.5625 * t * t + 0.984375
    return t


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def lerp_state(s1: EyeState, s2: EyeState, t: float,
               easing: EasingType = EasingType.EASE_IN_OUT) -> EyeState:
    # ELASTIC / BOUNCE easings overshoot past 1.0, so every channel must be
    # clamped to its valid range before reaching the renderer.
    et = ease(t, easing)
    # Discrete (non-interpolable) channels switch at the halfway point.
    disc = s2 if et >= 0.5 else s1
    return EyeState(
        gaze_x    = _clamp(lerp(s1.gaze_x,     s2.gaze_x,     et), -1.0, 1.0),
        gaze_y    = _clamp(lerp(s1.gaze_y,     s2.gaze_y,     et), -1.0, 1.0),
        upper_lid = _clamp(lerp(s1.upper_lid,  s2.upper_lid,  et),  0.0, 1.0),
        lower_lid = _clamp(lerp(s1.lower_lid,  s2.lower_lid,  et),  0.0, 1.0),
        pupil_size= _clamp(lerp(s1.pupil_size, s2.pupil_size, et),  0.0, 1.0),
        iris_color= tuple(int(_clamp(lerp(a, b, et), 0, 255))
                          for a, b in zip(s1.iris_color, s2.iris_color)),
        squint    = _clamp(lerp(s1.squint,     s2.squint,     et),  0.0, 1.0),
        eyebrow   = _clamp(lerp(s1.eyebrow,    s2.eyebrow,    et), -1.0, 1.0),
        lid_tilt  = _clamp(lerp(s1.lid_tilt,   s2.lid_tilt,   et), -1.0, 1.0),
        lower_lid_curve = _clamp(lerp(s1.lower_lid_curve,
                                      s2.lower_lid_curve,     et),  0.0, 1.0),
        pupil_style = disc.pupil_style,
        overlay     = disc.overlay,
    )


@dataclass
class Keyframe:
    state:    EyeState
    duration: float
    easing:   EasingType = EasingType.EASE_IN_OUT
    hold:     float = 0.0


# Channel groups an animation can control. Channels NOT in an animation's
# `channels` set are taken from the base state every frame, so e.g. a
# gaze-only glance keeps the active emotion's lids, squint and iris color.
CHANNEL_ATTRS = {
    'gaze':    ('gaze_x', 'gaze_y'),
    'lids':    ('upper_lid', 'lower_lid', 'lid_tilt', 'lower_lid_curve'),
    'pupil':   ('pupil_size', 'pupil_style'),
    'iris':    ('iris_color',),
    'squint':  ('squint',),
    'eyebrow': ('eyebrow',),
    'overlay': ('overlay',),
}


@dataclass
class Animation:
    name:            str
    left_keyframes:  List[Keyframe]
    right_keyframes: Optional[List[Keyframe]] = None  # None = mirror left
    loop:            bool = False
    on_complete:     Optional[Callable] = None
    channels:        Optional[frozenset] = None       # None = controls everything


class AnimationEngine:
    """Drives both eyes through animations. Thread-safe."""

    def __init__(self, fps: int = 30):
        self.fps = fps
        self.dt  = 1.0 / fps

        self.left_state  = EyeState()
        self.right_state = EyeState()

        self._lock          = threading.Lock()
        self._current_anim  = None
        self._anim_segment  = 0
        self._segment_time  = 0.0
        self._holding       = False
        self._hold_end      = 0.0

        # Base (idle) state -- stores BOTH gaze and expression.
        # Expressions set here persist through blinks and micro-saccades.
        self._base_left  = EyeState()
        self._base_right = EyeState()

        # Gaze pursuit target -- the base gaze eases toward this each tick
        self._gaze_target_x = 0.0
        self._gaze_target_y = 0.0

        # Micro-saccade render offset (decays back to 0, never accumulates)
        self._sacc_x = 0.0
        self._sacc_y = 0.0

        # State the current animation started from (avoids snapping when
        # an animation interrupts another mid-flight)
        self._entry_left  = EyeState()
        self._entry_right = EyeState()

        self._next_blink  = time.time() + self._rand_blink_interval()
        self._running     = False
        self._thread      = None

    # ------------------------------------------------------------------ public

    def start(self):
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def play(self, anim: Animation, interrupt: bool = True):
        with self._lock:
            if interrupt or self._current_anim is None:
                self._entry_left   = self._copy_state(self.left_state)
                self._entry_right  = self._copy_state(self.right_state)
                self._current_anim = anim
                self._anim_segment = 0
                self._segment_time = 0.0
                self._holding      = False

    def set_base_gaze(self, gaze_x: float, gaze_y: float):
        """Set the persistent gaze target; the eyes ease toward it (smooth pursuit)."""
        with self._lock:
            gaze_x = max(-1.0, min(1.0, gaze_x))
            gaze_y = max(-1.0, min(1.0, gaze_y))
            # Large gaze shifts often come with a reflex blink in humans
            shift = math.hypot(gaze_x - self._gaze_target_x,
                               gaze_y - self._gaze_target_y)
            if shift > 0.55 and random.random() < 0.35 and self._current_anim is None:
                self._next_blink = time.time()
            self._gaze_target_x = gaze_x
            self._gaze_target_y = gaze_y

    def set_base_expression(self, state: EyeState):
        """
        Persist an emotional expression into the base state.
        Gaze is preserved from the current base; all other properties
        (lids, squint, pupil, iris color, eyebrow) are taken from `state`.
        Blinks and idle movements will respect this expression until reset.
        """
        with self._lock:
            for attr in ('upper_lid', 'lower_lid', 'pupil_size',
                         'iris_color', 'squint', 'eyebrow',
                         'lid_tilt', 'lower_lid_curve', 'pupil_style', 'overlay'):
                val = getattr(state, attr)
                setattr(self._base_left,  attr, val)
                setattr(self._base_right, attr, val)

    def get_states(self):
        with self._lock:
            return self.left_state, self.right_state

    def cancel_if(self, name: str):
        """If the currently playing animation has this name, ease back to the
        base state (used e.g. to end the looping 'speaking' animation)."""
        with self._lock:
            if self._current_anim is not None and self._current_anim.name == name:
                self._start_return_to_base()

    # ----------------------------------------------------------------- private

    @staticmethod
    def _rand_blink_interval() -> float:
        """
        Irregular blink cadence mimicking human eyes.
        Mostly 2.5-7 s, but ~18 % of the time a long 9-16 s pause.
        """
        if random.random() < 0.18:
            return random.uniform(9.0, 16.0)
        return random.uniform(2.5, 7.0)

    def _loop(self):
        while self._running:
            t0 = time.time()
            self._tick()
            elapsed  = time.time() - t0
            sleep_t  = self.dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _tick(self):
        now = time.time()

        with self._lock:
            # Smooth pursuit: base gaze eases toward its target every tick
            self._update_gaze_pursuit()

            # Auto-blink when idle
            if self._current_anim is None and now >= self._next_blink:
                self._next_blink = now + self._rand_blink_interval()
                self._trigger_auto_blink()

            # Idle render -- base state plus micro-saccades and pupil hippus
            if self._current_anim is None:
                self._update_micro_saccade()
                self.left_state  = self._idle_state(self._base_left,  now)
                self.right_state = self._idle_state(self._base_right, now)
                return

            anim = self._current_anim
            lkf  = anim.left_keyframes
            rkf  = anim.right_keyframes or anim.left_keyframes

            if self._anim_segment >= len(lkf):
                if anim.loop:
                    self._anim_segment = 0
                    self._segment_time = 0.0
                    # Loop continues from its own last keyframe, not from base
                    self._entry_left  = self._copy_state(lkf[-1].state)
                    self._entry_right = self._copy_state(rkf[-1].state)
                else:
                    if anim.on_complete:
                        anim.on_complete()
                    self._current_anim = None
                    # Ease back to the base state instead of snapping
                    if anim.name != 'return_to_base':
                        self._start_return_to_base()
                return

            seg_l = lkf[self._anim_segment]
            seg_r = rkf[min(self._anim_segment, len(rkf) - 1)]

            if self._holding:
                if now >= self._hold_end:
                    self._holding      = False
                    self._anim_segment += 1
                    self._segment_time = 0.0
                else:
                    self.left_state  = self._merge_with_base(seg_l.state, self._base_left,  anim.channels)
                    self.right_state = self._merge_with_base(seg_r.state, self._base_right, anim.channels)
                return

            t = self._segment_time / max(seg_l.duration, 0.001)

            if self._anim_segment == 0:
                prev_l = self._entry_left
                prev_r = self._entry_right
            else:
                prev_l = lkf[self._anim_segment - 1].state
                prev_r = rkf[min(self._anim_segment - 1, len(rkf) - 1)].state

            raw_l = lerp_state(prev_l, seg_l.state, t, seg_l.easing)
            raw_r = lerp_state(prev_r, seg_r.state, t, seg_r.easing)
            self.left_state  = self._merge_with_base(raw_l, self._base_left,  anim.channels)
            self.right_state = self._merge_with_base(raw_r, self._base_right, anim.channels)

            self._segment_time += self.dt

            if self._segment_time >= seg_l.duration:
                self.left_state  = self._merge_with_base(seg_l.state, self._base_left,  anim.channels)
                self.right_state = self._merge_with_base(seg_r.state, self._base_right, anim.channels)
                if seg_l.hold > 0:
                    self._holding  = True
                    self._hold_end = now + seg_l.hold
                else:
                    self._anim_segment += 1
                    self._segment_time  = 0.0

    @staticmethod
    def _copy_state(src: EyeState) -> EyeState:
        return EyeState(
            gaze_x    = src.gaze_x,
            gaze_y    = src.gaze_y,
            upper_lid = src.upper_lid,
            lower_lid = src.lower_lid,
            pupil_size= src.pupil_size,
            iris_color= src.iris_color,
            squint    = src.squint,
            eyebrow   = src.eyebrow,
            lid_tilt  = src.lid_tilt,
            lower_lid_curve = src.lower_lid_curve,
            pupil_style     = src.pupil_style,
            overlay         = src.overlay,
        )

    def _merge_with_base(self, state: EyeState, base: EyeState,
                         channels: Optional[frozenset]) -> EyeState:
        """Take `channels` from the animated state, everything else from base."""
        if channels is None:
            return state
        merged = self._copy_state(base)
        for ch in channels:
            for attr in CHANNEL_ATTRS.get(ch, ()):
                setattr(merged, attr, getattr(state, attr))
        return merged

    def _start_return_to_base(self, duration: float = 0.25):
        """Ease from wherever the last animation ended back to the base state."""
        kf_l = [Keyframe(self._copy_state(self._base_left),  duration, EasingType.EASE_OUT)]
        kf_r = [Keyframe(self._copy_state(self._base_right), duration, EasingType.EASE_OUT)]
        self._entry_left   = self._copy_state(self.left_state)
        self._entry_right  = self._copy_state(self.right_state)
        self._current_anim = Animation('return_to_base',
                                       left_keyframes=kf_l, right_keyframes=kf_r)
        self._anim_segment = 0
        self._segment_time = 0.0
        self._holding      = False

    def _update_gaze_pursuit(self):
        """Exponential easing of the base gaze toward its target."""
        alpha = 1.0 - math.exp(-self.dt / 0.09)
        for base in (self._base_left, self._base_right):
            base.gaze_x += (self._gaze_target_x - base.gaze_x) * alpha
            base.gaze_y += (self._gaze_target_y - base.gaze_y) * alpha

    def _idle_state(self, base: EyeState, now: float) -> EyeState:
        """Base state plus micro-saccade offset and pupil hippus (liveliness)."""
        st = self._copy_state(base)
        st.gaze_x = max(-1.0, min(1.0, st.gaze_x + self._sacc_x))
        st.gaze_y = max(-1.0, min(1.0, st.gaze_y + self._sacc_y))
        # Hippus: the pupil never sits perfectly still
        hippus = 0.02 * math.sin(now * 2.1) + 0.015 * math.sin(now * 0.7)
        st.pupil_size = max(0.0, min(1.0, st.pupil_size + hippus))
        return st

    def _trigger_auto_blink(self):
        """
        Queue a blink that returns to the CURRENT base expression.
        Variety: 12 % double, 18 % slow, 70 % normal.
        """
        from .eye_renderer import EyeState as _ES

        # Closed state inherits the current expression except lids
        closed = _ES(
            gaze_x    = self._base_left.gaze_x,
            gaze_y    = self._base_left.gaze_y,
            upper_lid = 0.0,
            lower_lid = 0.8,
            pupil_size= max(0.2, self._base_left.pupil_size * 0.6),
            iris_color= self._base_left.iris_color,
            squint    = self._base_left.squint,
            eyebrow   = self._base_left.eyebrow,
            lid_tilt  = self._base_left.lid_tilt,
            lower_lid_curve = 0.0,
            pupil_style     = self._base_left.pupil_style,
            overlay         = self._base_left.overlay,
        )
        # Return-to-base state (current expression fully open)
        rtn = self._copy_state(self._base_left)
        rtn.upper_lid = max(self._base_left.upper_lid, 0.85)
        rtn.lower_lid = 0.0

        r = random.random()
        if r < 0.12:        # double blink
            kf = [
                Keyframe(closed, 0.06, EasingType.EASE_IN),
                Keyframe(rtn,    0.06, EasingType.EASE_OUT, hold=0.07),
                Keyframe(closed, 0.06, EasingType.EASE_IN),
                Keyframe(rtn,    0.10, EasingType.EASE_OUT),
            ]
        elif r < 0.30:      # slow sleepy blink
            kf = [
                Keyframe(closed, 0.25, EasingType.EASE_IN),
                Keyframe(rtn,    0.35, EasingType.EASE_OUT),
            ]
        else:               # normal blink
            kf = [
                Keyframe(closed, 0.07, EasingType.EASE_IN),
                Keyframe(rtn,    0.11, EasingType.EASE_OUT),
            ]

        self._entry_left   = self._copy_state(self.left_state)
        self._entry_right  = self._copy_state(self.right_state)
        self._current_anim = Animation("auto_blink", left_keyframes=kf)
        self._anim_segment = 0
        self._segment_time = 0.0
        self._holding      = False

    def _update_micro_saccade(self):
        """
        Tiny involuntary gaze jumps -- roughly every 1.5 s at 30 fps.
        Applied as a render-time offset that decays back to zero, so the
        base gaze never drifts away from its target.
        """
        if random.random() < 0.022:
            self._sacc_x = random.uniform(-0.035, 0.035)
            self._sacc_y = random.uniform(-0.035, 0.035)
        else:
            self._sacc_x *= 0.92
            self._sacc_y *= 0.92
