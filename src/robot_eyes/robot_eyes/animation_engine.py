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
    LINEAR = auto()
    EASE_IN = auto()
    EASE_OUT = auto()
    EASE_IN_OUT = auto()
    ELASTIC = auto()
    BOUNCE = auto()


def ease(t: float, easing: EasingType) -> float:
    """Apply easing function to normalized time t ∈ [0,1]."""
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
        p = 0.3
        s = p / 4.0
        return (2.0 ** (-10 * t)) * math.sin((t - s) * (2 * math.pi) / p) + 1.0
    elif easing == EasingType.BOUNCE:
        if t < 1 / 2.75:
            return 7.5625 * t * t
        elif t < 2 / 2.75:
            t -= 1.5 / 2.75
            return 7.5625 * t * t + 0.75
        elif t < 2.5 / 2.75:
            t -= 2.25 / 2.75
            return 7.5625 * t * t + 0.9375
        else:
            t -= 2.625 / 2.75
            return 7.5625 * t * t + 0.984375
    return t


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def lerp_state(s1: EyeState, s2: EyeState, t: float, easing: EasingType = EasingType.EASE_IN_OUT) -> EyeState:
    """Interpolate between two EyeStates."""
    et = ease(t, easing)
    return EyeState(
        gaze_x=lerp(s1.gaze_x, s2.gaze_x, et),
        gaze_y=lerp(s1.gaze_y, s2.gaze_y, et),
        upper_lid=lerp(s1.upper_lid, s2.upper_lid, et),
        lower_lid=lerp(s1.lower_lid, s2.lower_lid, et),
        pupil_size=lerp(s1.pupil_size, s2.pupil_size, et),
        iris_color=tuple(int(lerp(a, b, et)) for a, b in zip(s1.iris_color, s2.iris_color)),
        squint=lerp(s1.squint, s2.squint, et),
        eyebrow=lerp(s1.eyebrow, s2.eyebrow, et),
    )


@dataclass
class Keyframe:
    """A single keyframe in an animation sequence."""
    state: EyeState
    duration: float          # seconds to reach this keyframe from previous
    easing: EasingType = EasingType.EASE_IN_OUT
    hold: float = 0.0        # seconds to hold at this keyframe after arriving


@dataclass
class Animation:
    """A sequence of keyframes forming a complete animation."""
    name: str
    left_keyframes: List[Keyframe]
    right_keyframes: Optional[List[Keyframe]] = None  # None = mirror left
    loop: bool = False
    on_complete: Optional[Callable] = None


class AnimationEngine:
    """
    Drives both eyes through animations at a target frame rate.
    Thread-safe: can accept new animations while running.
    """

    def __init__(self, fps: int = 30):
        self.fps = fps
        self.dt = 1.0 / fps

        # Current rendered states (what gets drawn)
        self.left_state = EyeState()
        self.right_state = EyeState()

        # Animation queue and current animation
        self._lock = threading.Lock()
        self._current_anim: Optional[Animation] = None
        self._anim_time: float = 0.0       # time within current animation
        self._anim_segment: int = 0         # current keyframe index
        self._segment_time: float = 0.0     # time within current segment
        self._holding: bool = False
        self._hold_end: float = 0.0

        # Base (idle) state
        self._base_left = EyeState()
        self._base_right = EyeState()

        # Blink timer
        self._next_blink = time.time() + random.uniform(2.0, 5.0)
        self._blink_queued = False

        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def play(self, anim: Animation, interrupt: bool = True):
        """Queue or immediately play an animation."""
        with self._lock:
            if interrupt or self._current_anim is None:
                self._current_anim = anim
                self._anim_segment = 0
                self._segment_time = 0.0
                self._holding = False

    def set_base_gaze(self, gaze_x: float, gaze_y: float):
        """Continuously update the base gaze direction (e.g., from face tracking)."""
        with self._lock:
            self._base_left.gaze_x = gaze_x
            self._base_left.gaze_y = gaze_y
            self._base_right.gaze_x = gaze_x
            self._base_right.gaze_y = gaze_y

    def get_states(self):
        """Return (left_state, right_state) thread-safely."""
        with self._lock:
            return self.left_state, self.right_state

    def _loop(self):
        while self._running:
            t0 = time.time()
            self._tick()
            elapsed = time.time() - t0
            sleep_time = self.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _tick(self):
        now = time.time()

        with self._lock:
            # Auto-blink when idle
            if self._current_anim is None and now >= self._next_blink:
                self._next_blink = now + random.uniform(2.5, 6.0)
                # Schedule a blink by setting a quick blink animation
                self._trigger_auto_blink()

            if self._current_anim is None:
                # Idle: micro saccades
                self._apply_micro_saccade(now)
                self.left_state = EyeState(
                    gaze_x=self._base_left.gaze_x,
                    gaze_y=self._base_left.gaze_y,
                    pupil_size=self._base_left.pupil_size,
                    iris_color=self._base_left.iris_color,
                )
                self.right_state = EyeState(
                    gaze_x=self._base_right.gaze_x,
                    gaze_y=self._base_right.gaze_y,
                    pupil_size=self._base_right.pupil_size,
                    iris_color=self._base_right.iris_color,
                )
                return

            anim = self._current_anim
            lkf = anim.left_keyframes
            rkf = anim.right_keyframes or anim.left_keyframes

            if self._anim_segment >= len(lkf):
                if anim.loop:
                    self._anim_segment = 0
                    self._segment_time = 0.0
                else:
                    if anim.on_complete:
                        anim.on_complete()
                    self._current_anim = None
                return

            seg_l = lkf[self._anim_segment]
            seg_r = rkf[min(self._anim_segment, len(rkf) - 1)]

            if self._holding:
                if now >= self._hold_end:
                    self._holding = False
                    self._anim_segment += 1
                    self._segment_time = 0.0
                else:
                    # Stay at current keyframe
                    self.left_state = seg_l.state
                    self.right_state = seg_r.state
                    return

            # Normal interpolation
            t = self._segment_time / max(seg_l.duration, 0.001)

            if self._anim_segment == 0:
                prev_l = self._base_left
                prev_r = self._base_right
            else:
                prev_l = lkf[self._anim_segment - 1].state
                prev_r = rkf[min(self._anim_segment - 1, len(rkf) - 1)].state

            self.left_state = lerp_state(prev_l, seg_l.state, t, seg_l.easing)
            self.right_state = lerp_state(prev_r, seg_r.state, t, seg_r.easing)

            self._segment_time += self.dt

            if self._segment_time >= seg_l.duration:
                self.left_state = seg_l.state
                self.right_state = seg_r.state
                if seg_l.hold > 0:
                    self._holding = True
                    self._hold_end = now + seg_l.hold
                else:
                    self._anim_segment += 1
                    self._segment_time = 0.0

    def _trigger_auto_blink(self):
        """Internal: queue a blink animation."""
        from .behaviors import BehaviorLibrary
        blink = BehaviorLibrary.blink()
        # Play without re-locking (already holding lock)
        self._current_anim = blink
        self._anim_segment = 0
        self._segment_time = 0.0
        self._holding = False

    def _apply_micro_saccade(self, now: float):
        """Tiny random eye movements to look alive."""
        # Very subtle: ±2% of range, change every ~200ms on average
        if random.random() < 0.005:  # ~5 times per second at 30fps → ~every 6s, subtle
            jitter = 0.04
            self._base_left.gaze_x += random.uniform(-jitter, jitter)
            self._base_left.gaze_y += random.uniform(-jitter, jitter)
            # Clamp
            self._base_left.gaze_x = max(-0.8, min(0.8, self._base_left.gaze_x))
            self._base_left.gaze_y = max(-0.8, min(0.8, self._base_left.gaze_y))
            self._base_right.gaze_x = self._base_left.gaze_x
            self._base_right.gaze_y = self._base_left.gaze_y
