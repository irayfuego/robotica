#!/usr/bin/env python3
"""
Behavior library for robot eyes.
Contains all pre-defined animations and behaviors.
"""

import random
from typing import Optional, Tuple
from .eye_renderer import EyeState
from .animation_engine import Animation, Keyframe, EasingType


# ---------------------------------------------------------------------------
# Canonical "open eye" state (reference for building animations from)
# ---------------------------------------------------------------------------
OPEN = EyeState(
    gaze_x=0.0, gaze_y=0.0,
    upper_lid=1.0, lower_lid=0.0,
    pupil_size=0.5,
    iris_color=(60, 120, 200),
    squint=0.0
)

CLOSED = EyeState(
    gaze_x=0.0, gaze_y=0.0,
    upper_lid=0.0, lower_lid=1.0,
    pupil_size=0.5,
    iris_color=(60, 120, 200),
    squint=0.0
)

HALF_CLOSED = EyeState(
    gaze_x=0.0, gaze_y=0.0,
    upper_lid=0.35, lower_lid=0.2,
    pupil_size=0.45,
    iris_color=(60, 120, 200),
    squint=0.1
)


def _open(gaze_x=0.0, gaze_y=0.0, pupil=0.5, iris_color=(60, 120, 200)) -> EyeState:
    return EyeState(
        gaze_x=gaze_x, gaze_y=gaze_y,
        upper_lid=1.0, lower_lid=0.0,
        pupil_size=pupil,
        iris_color=iris_color,
        squint=0.0
    )


def _closed(gaze_x=0.0, gaze_y=0.0) -> EyeState:
    return EyeState(
        gaze_x=gaze_x, gaze_y=gaze_y,
        upper_lid=0.0, lower_lid=0.8,
        pupil_size=0.3,
        iris_color=(60, 120, 200),
    )


class BehaviorLibrary:
    """
    Factory class for all robot eye behaviors.
    Returns Animation objects ready to be played by AnimationEngine.
    """

    # -----------------------------------------------------------------------
    # BASIC BEHAVIORS
    # -----------------------------------------------------------------------

    @staticmethod
    def blink(speed: float = 1.0) -> Animation:
        """Standard bilateral blink."""
        d = 0.07 / speed
        kf = [
            Keyframe(_closed(),  duration=d,     easing=EasingType.EASE_IN),
            Keyframe(_open(),    duration=d * 1.5, easing=EasingType.EASE_OUT),
        ]
        return Animation("blink", left_keyframes=kf)

    @staticmethod
    def double_blink() -> Animation:
        """Two quick blinks in succession."""
        d = 0.06
        kf = [
            Keyframe(_closed(), duration=d,       easing=EasingType.EASE_IN),
            Keyframe(_open(),   duration=d,       easing=EasingType.EASE_OUT, hold=0.08),
            Keyframe(_closed(), duration=d,       easing=EasingType.EASE_IN),
            Keyframe(_open(),   duration=d * 1.5, easing=EasingType.EASE_OUT),
        ]
        return Animation("double_blink", left_keyframes=kf)

    @staticmethod
    def slow_blink() -> Animation:
        """Slow, sleepy blink."""
        kf = [
            Keyframe(_closed(), duration=0.25, easing=EasingType.EASE_IN),
            Keyframe(_open(),   duration=0.35, easing=EasingType.EASE_OUT),
        ]
        return Animation("slow_blink", left_keyframes=kf)

    @staticmethod
    def wink(eye: str = "right") -> Animation:
        """Wink one eye while keeping the other open."""
        closed_state = _closed()
        open_state = _open()

        wink_kf = [
            Keyframe(closed_state, duration=0.08, easing=EasingType.EASE_IN),
            Keyframe(open_state,   duration=0.15, easing=EasingType.EASE_OUT),
        ]
        hold_kf = [
            Keyframe(open_state, duration=0.001),
            Keyframe(open_state, duration=0.001),
        ]

        if eye == "right":
            return Animation("wink_right", left_keyframes=hold_kf, right_keyframes=wink_kf)
        else:
            return Animation("wink_left", left_keyframes=wink_kf, right_keyframes=hold_kf)

    # -----------------------------------------------------------------------
    # GAZE BEHAVIORS
    # -----------------------------------------------------------------------

    @staticmethod
    def look_at(gaze_x: float, gaze_y: float, duration: float = 0.3) -> Animation:
        """Move gaze to a specific direction."""
        target = _open(gaze_x=gaze_x, gaze_y=gaze_y)
        kf = [Keyframe(target, duration=duration, easing=EasingType.EASE_IN_OUT)]
        return Animation("look_at", left_keyframes=kf)

    @staticmethod
    def look_left(amount: float = 0.7) -> Animation:
        return BehaviorLibrary.look_at(-amount, 0.0)

    @staticmethod
    def look_right(amount: float = 0.7) -> Animation:
        return BehaviorLibrary.look_at(amount, 0.0)

    @staticmethod
    def look_up(amount: float = 0.6) -> Animation:
        return BehaviorLibrary.look_at(0.0, -amount)

    @staticmethod
    def look_down(amount: float = 0.6) -> Animation:
        return BehaviorLibrary.look_at(0.0, amount)

    @staticmethod
    def look_center() -> Animation:
        return BehaviorLibrary.look_at(0.0, 0.0)

    @staticmethod
    def scan_horizontal() -> Animation:
        """Scan left then right, return to center (like searching)."""
        kf = [
            Keyframe(_open(gaze_x=-0.8),  duration=0.4, easing=EasingType.EASE_IN_OUT, hold=0.2),
            Keyframe(_open(gaze_x=0.8),   duration=0.6, easing=EasingType.EASE_IN_OUT, hold=0.2),
            Keyframe(_open(gaze_x=0.0),   duration=0.4, easing=EasingType.EASE_IN_OUT),
        ]
        return Animation("scan_horizontal", left_keyframes=kf)

    @staticmethod
    def saccade(gaze_x: float, gaze_y: float) -> Animation:
        """Fast saccadic eye movement (no easing, instant snap)."""
        target = _open(gaze_x=gaze_x, gaze_y=gaze_y)
        kf = [Keyframe(target, duration=0.04, easing=EasingType.LINEAR)]
        return Animation("saccade", left_keyframes=kf)

    @staticmethod
    def track_face(gaze_x: float, gaze_y: float) -> Animation:
        """
        Smooth pursuit - used when following a detected face.
        Very short duration so it feels responsive.
        """
        target = _open(gaze_x=gaze_x, gaze_y=gaze_y)
        kf = [Keyframe(target, duration=0.12, easing=EasingType.EASE_OUT)]
        return Animation("track_face", left_keyframes=kf)

    @staticmethod
    def look_away_shy() -> Animation:
        """Look away shyly then back."""
        kf = [
            Keyframe(_open(gaze_x=-0.6, gaze_y=0.3), duration=0.3, hold=0.8),
            Keyframe(_open(gaze_x=0.0), duration=0.4, easing=EasingType.EASE_IN_OUT),
        ]
        return Animation("look_away_shy", left_keyframes=kf)

    # -----------------------------------------------------------------------
    # EMOTIONAL EXPRESSIONS
    # -----------------------------------------------------------------------

    @staticmethod
    def happy() -> Animation:
        """Happy expression: eyes slightly squinted, looking straight."""
        happy_state = EyeState(
            gaze_x=0.0, gaze_y=0.0,
            upper_lid=0.85, lower_lid=0.25,
            pupil_size=0.6,
            iris_color=(80, 180, 120),
            squint=0.3,
        )
        kf = [
            Keyframe(happy_state, duration=0.2, easing=EasingType.EASE_OUT, hold=1.5),
            Keyframe(_open(),     duration=0.3, easing=EasingType.EASE_IN_OUT),
        ]
        return Animation("happy", left_keyframes=kf)

    @staticmethod
    def sad() -> Animation:
        """Sad expression: droopy eyelids, looking down."""
        sad_state = EyeState(
            gaze_x=0.0, gaze_y=0.4,
            upper_lid=0.55, lower_lid=0.1,
            pupil_size=0.35,
            iris_color=(60, 80, 160),
            squint=0.0,
        )
        kf = [
            Keyframe(sad_state, duration=0.5, easing=EasingType.EASE_IN, hold=2.0),
            Keyframe(_open(),   duration=0.6, easing=EasingType.EASE_IN_OUT),
        ]
        return Animation("sad", left_keyframes=kf)

    @staticmethod
    def surprised() -> Animation:
        """Wide open eyes, pupils dilated."""
        surprised_state = EyeState(
            gaze_x=0.0, gaze_y=0.0,
            upper_lid=1.0, lower_lid=0.0,
            pupil_size=0.9,
            iris_color=(80, 150, 220),
            squint=0.0,
        )
        kf = [
            Keyframe(surprised_state, duration=0.08, easing=EasingType.ELASTIC, hold=1.0),
            Keyframe(_open(),         duration=0.5,  easing=EasingType.EASE_IN_OUT),
        ]
        return Animation("surprised", left_keyframes=kf)

    @staticmethod
    def angry() -> Animation:
        """Angry: inner brow down, squinting."""
        angry_state = EyeState(
            gaze_x=0.0, gaze_y=0.0,
            upper_lid=0.65, lower_lid=0.3,
            pupil_size=0.25,
            iris_color=(200, 60, 40),
            squint=0.5,
            eyebrow=-0.8,
        )
        kf = [
            Keyframe(angry_state, duration=0.15, easing=EasingType.EASE_IN, hold=1.5),
            Keyframe(_open(),     duration=0.4,  easing=EasingType.EASE_IN_OUT),
        ]
        return Animation("angry", left_keyframes=kf)

    @staticmethod
    def suspicious() -> Animation:
        """One eye squinted, head tilted look."""
        suspicious_l = EyeState(
            gaze_x=0.2, gaze_y=0.0,
            upper_lid=0.5, lower_lid=0.3,
            pupil_size=0.3,
            iris_color=(100, 100, 60),
            squint=0.6,
        )
        suspicious_r = EyeState(
            gaze_x=0.2, gaze_y=0.0,
            upper_lid=0.85, lower_lid=0.0,
            pupil_size=0.45,
            iris_color=(100, 100, 60),
            squint=0.1,
        )
        lkf = [
            Keyframe(suspicious_l, duration=0.2, hold=1.5),
            Keyframe(_open(),      duration=0.3),
        ]
        rkf = [
            Keyframe(suspicious_r, duration=0.2, hold=1.5),
            Keyframe(_open(),      duration=0.3),
        ]
        return Animation("suspicious", left_keyframes=lkf, right_keyframes=rkf)

    @staticmethod
    def tired() -> Animation:
        """Sleepy drooping eyelids."""
        tired_state = EyeState(
            gaze_x=0.0, gaze_y=0.2,
            upper_lid=0.45, lower_lid=0.1,
            pupil_size=0.4,
            iris_color=(60, 100, 160),
            squint=0.15,
        )
        kf = [
            Keyframe(tired_state, duration=1.0, easing=EasingType.EASE_IN, hold=2.0),
            Keyframe(_open(),     duration=0.8, easing=EasingType.EASE_OUT),
        ]
        return Animation("tired", left_keyframes=kf)

    @staticmethod
    def love() -> Animation:
        """Heart eyes effect (approximated with dilated pupils + warm color)."""
        love_state = EyeState(
            gaze_x=0.0, gaze_y=-0.1,
            upper_lid=0.9, lower_lid=0.1,
            pupil_size=0.85,
            iris_color=(220, 80, 120),
            squint=0.2,
        )
        kf = [
            Keyframe(love_state, duration=0.3, easing=EasingType.BOUNCE, hold=2.0),
            Keyframe(_open(),    duration=0.5, easing=EasingType.EASE_IN_OUT),
        ]
        return Animation("love", left_keyframes=kf)

    @staticmethod
    def confused() -> Animation:
        """One eye open, one squinted, with a sideways glance."""
        conf_l = EyeState(
            gaze_x=0.3, gaze_y=-0.2,
            upper_lid=0.9, lower_lid=0.0,
            pupil_size=0.5,
            iris_color=(100, 160, 80),
            squint=0.0,
        )
        conf_r = EyeState(
            gaze_x=0.3, gaze_y=-0.2,
            upper_lid=0.6, lower_lid=0.15,
            pupil_size=0.5,
            iris_color=(100, 160, 80),
            squint=0.4,
        )
        lkf = [
            Keyframe(conf_l, duration=0.25, hold=1.5),
            Keyframe(_open(), duration=0.3),
        ]
        rkf = [
            Keyframe(conf_r, duration=0.25, hold=1.5),
            Keyframe(_open(), duration=0.3),
        ]
        return Animation("confused", left_keyframes=lkf, right_keyframes=rkf)

    # -----------------------------------------------------------------------
    # SLEEP / WAKE
    # -----------------------------------------------------------------------

    @staticmethod
    def fall_asleep() -> Animation:
        """Gradually close eyes as if falling asleep."""
        kf = [
            Keyframe(HALF_CLOSED, duration=1.5, easing=EasingType.EASE_IN, hold=0.5),
            Keyframe(_closed(),   duration=1.0, easing=EasingType.EASE_IN),
        ]
        return Animation("fall_asleep", left_keyframes=kf)

    @staticmethod
    def wake_up() -> Animation:
        """Wake up from sleep."""
        kf = [
            Keyframe(HALF_CLOSED, duration=0.8, easing=EasingType.EASE_OUT, hold=0.3),
            Keyframe(_open(),     duration=0.6, easing=EasingType.ELASTIC),
        ]
        return Animation("wake_up", left_keyframes=kf)

    @staticmethod
    def sleeping_loop() -> Animation:
        """Looping slow breathing while asleep (subtle lid movement)."""
        asleep = EyeState(
            gaze_x=0.0, gaze_y=0.3,
            upper_lid=0.05, lower_lid=0.6,
            pupil_size=0.2,
            iris_color=(50, 80, 140),
        )
        asleep2 = EyeState(
            gaze_x=0.0, gaze_y=0.3,
            upper_lid=0.0, lower_lid=0.7,
            pupil_size=0.2,
            iris_color=(50, 80, 140),
        )
        kf = [
            Keyframe(asleep,  duration=1.8, easing=EasingType.EASE_IN_OUT),
            Keyframe(asleep2, duration=2.2, easing=EasingType.EASE_IN_OUT),
        ]
        return Animation("sleeping_loop", left_keyframes=kf, loop=True)

    # -----------------------------------------------------------------------
    # ATTENTION / INTERACTION
    # -----------------------------------------------------------------------

    @staticmethod
    def notice(gaze_x: float = 0.5, gaze_y: float = 0.0) -> Animation:
        """Quick look toward something that caught attention."""
        target = _open(gaze_x=gaze_x, gaze_y=gaze_y, pupil=0.7)
        kf = [
            Keyframe(target,  duration=0.08, easing=EasingType.LINEAR, hold=0.4),
            Keyframe(_open(), duration=0.3,  easing=EasingType.EASE_IN_OUT),
        ]
        return Animation("notice", left_keyframes=kf)

    @staticmethod
    def thinking() -> Animation:
        """Eyes moving as if thinking (upper-right is associated with imagination)."""
        think = _open(gaze_x=0.5, gaze_y=-0.4)
        kf = [
            Keyframe(think,                      duration=0.25, hold=0.8),
            Keyframe(_open(gaze_x=0.2, gaze_y=-0.3), duration=0.3, hold=0.4),
            Keyframe(_open(),                    duration=0.4),
        ]
        return Animation("thinking", left_keyframes=kf)

    @staticmethod
    def dizzy() -> Animation:
        """Eyes spinning slightly (confused/dizzy)."""
        kf = [
            Keyframe(_open(gaze_x=0.8,  gaze_y=0.0),  duration=0.15),
            Keyframe(_open(gaze_x=0.0,  gaze_y=-0.8), duration=0.15),
            Keyframe(_open(gaze_x=-0.8, gaze_y=0.0),  duration=0.15),
            Keyframe(_open(gaze_x=0.0,  gaze_y=0.8),  duration=0.15),
            Keyframe(_open(gaze_x=0.8,  gaze_y=0.0),  duration=0.15),
            Keyframe(_open(gaze_x=0.0,  gaze_y=-0.8), duration=0.15),
            Keyframe(_open(gaze_x=-0.8, gaze_y=0.0),  duration=0.15),
            Keyframe(_open(gaze_x=0.0,  gaze_y=0.0),  duration=0.3, easing=EasingType.EASE_OUT),
        ]
        return Animation("dizzy", left_keyframes=kf)

    @staticmethod
    def roll_eyes() -> Animation:
        """Roll eyes (up and around)."""
        kf = [
            Keyframe(_open(gaze_x=0.0, gaze_y=-0.8),  duration=0.2),
            Keyframe(_open(gaze_x=0.8, gaze_y=-0.4),  duration=0.2),
            Keyframe(_open(gaze_x=0.8, gaze_y=0.4),   duration=0.2),
            Keyframe(_open(gaze_x=0.0, gaze_y=0.6),   duration=0.2),
            Keyframe(_open(gaze_x=-0.8, gaze_y=0.0),  duration=0.2),
            Keyframe(_open(gaze_x=0.0, gaze_y=0.0),   duration=0.3),
        ]
        return Animation("roll_eyes", left_keyframes=kf)

    @staticmethod
    def pupil_dilate(amount: float = 1.0) -> Animation:
        """Dilate pupils (excitement, low light)."""
        dilated = _open(pupil=amount)
        kf = [
            Keyframe(dilated, duration=0.3, easing=EasingType.EASE_OUT, hold=1.0),
            Keyframe(_open(), duration=0.5, easing=EasingType.EASE_IN_OUT),
        ]
        return Animation("pupil_dilate", left_keyframes=kf)

    @staticmethod
    def random_behavior() -> Animation:
        """Pick a random idle behavior."""
        behaviors = [
            BehaviorLibrary.blink,
            BehaviorLibrary.slow_blink,
            BehaviorLibrary.look_left,
            BehaviorLibrary.look_right,
            BehaviorLibrary.look_up,
            BehaviorLibrary.look_down,
            BehaviorLibrary.thinking,
            BehaviorLibrary.look_away_shy,
        ]
        return random.choice(behaviors)()


# ---------------------------------------------------------------------------
# Public behavior map (name -> factory callable)
# ---------------------------------------------------------------------------
BEHAVIOR_MAP = {
    "blink":          BehaviorLibrary.blink,
    "double_blink":   BehaviorLibrary.double_blink,
    "slow_blink":     BehaviorLibrary.slow_blink,
    "wink_right":     lambda: BehaviorLibrary.wink("right"),
    "wink_left":      lambda: BehaviorLibrary.wink("left"),
    "look_left":      BehaviorLibrary.look_left,
    "look_right":     BehaviorLibrary.look_right,
    "look_up":        BehaviorLibrary.look_up,
    "look_down":      BehaviorLibrary.look_down,
    "look_center":    BehaviorLibrary.look_center,
    "scan":           BehaviorLibrary.scan_horizontal,
    "thinking":       BehaviorLibrary.thinking,
    "roll_eyes":      BehaviorLibrary.roll_eyes,
    "dizzy":          BehaviorLibrary.dizzy,
    "look_away":      BehaviorLibrary.look_away_shy,
    "happy":          BehaviorLibrary.happy,
    "sad":            BehaviorLibrary.sad,
    "surprised":      BehaviorLibrary.surprised,
    "angry":          BehaviorLibrary.angry,
    "suspicious":     BehaviorLibrary.suspicious,
    "tired":          BehaviorLibrary.tired,
    "love":           BehaviorLibrary.love,
    "confused":       BehaviorLibrary.confused,
    "fall_asleep":    BehaviorLibrary.fall_asleep,
    "wake_up":        BehaviorLibrary.wake_up,
    "sleeping":       BehaviorLibrary.sleeping_loop,
    "dilate":         BehaviorLibrary.pupil_dilate,
    "notice":         BehaviorLibrary.notice,
    "random":         BehaviorLibrary.random_behavior,
    "saccade_right":  lambda: BehaviorLibrary.saccade(0.7, 0.0),
    "saccade_left":   lambda: BehaviorLibrary.saccade(-0.7, 0.0),
}
