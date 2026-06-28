#!/usr/bin/env python3
"""
Behavior library for robot eyes.
Emotion behaviors are PERSISTENT: they animate TO the state and stay there
until a new emotion or 'neutral' overrides them.
"""

import random
from .eye_renderer import EyeState
from .animation_engine import Animation, Keyframe, EasingType

# Channel masks: which EyeState channels a behavior controls.
# Unlisted channels come from the engine's base state, so glances and
# blinks no longer wipe out the active emotion or iris color.
CH_GAZE       = frozenset({'gaze'})
CH_LIDS       = frozenset({'lids'})
CH_PUPIL      = frozenset({'pupil'})
CH_GAZE_PUPIL = frozenset({'gaze', 'pupil'})
# "Pensar con el LLM": expresion propia y reconocible. Controla mirada, iris,
# pupila, parpados y borra el ceño (squint/eyebrow/lid_tilt) -> no hereda el
# rojo ni la "V" de una emocion 'angry' previa, y se distingue de cualquier
# emocion mientras espera al LLM.
CH_THINK      = frozenset({'gaze', 'iris', 'squint', 'eyebrow', 'lids', 'pupil'})


# ---------------------------------------------------------------------------
# Reference states
# ---------------------------------------------------------------------------

def _open(gaze_x=0.0, gaze_y=0.0, pupil=0.5, iris_color=(60, 120, 200)) -> EyeState:
    return EyeState(
        gaze_x=gaze_x, gaze_y=gaze_y,
        upper_lid=1.0, lower_lid=0.0,
        pupil_size=pupil, iris_color=iris_color,
        squint=0.0, eyebrow=0.0,
    )


def _thinking(gaze_x=0.0, gaze_y=0.0) -> EyeState:
    """Expresion de 'pensando con el LLM': parpados algo entornados, ceja
    ligeramente arqueada, pupila algo contraida e iris cian. Lectura clara de
    'procesando', distinta de cualquier emocion (ninguna usa cian)."""
    return EyeState(
        gaze_x=gaze_x, gaze_y=gaze_y,
        upper_lid=0.78, lower_lid=0.12,
        pupil_size=0.42, iris_color=(40, 180, 190),
        squint=0.0, eyebrow=0.3, lid_tilt=0.0,
    )


def _closed(gaze_x=0.0, gaze_y=0.0) -> EyeState:
    return EyeState(
        gaze_x=gaze_x, gaze_y=gaze_y,
        upper_lid=0.0, lower_lid=0.8,
        pupil_size=0.3, iris_color=(60, 120, 200),
    )


OPEN        = _open()
CLOSED      = _closed()
HALF_CLOSED = EyeState(
    upper_lid=0.35, lower_lid=0.2, pupil_size=0.45,
    iris_color=(60, 120, 200), squint=0.1,
)


# ---------------------------------------------------------------------------
# Emotion target states (module-level, importable for set_base_expression)
# ---------------------------------------------------------------------------

# lid_tilt: negative = inner corners down (angry "V"), positive = outer
# corners down (sad droop). lower_lid_curve: smiling ^_^ arc. overlay and
# pupil_style add animated decorations (tear, Zz, heart pupils...).
EMOTION_STATES = {
    "neutral": _open(),

    "happy": EyeState(
        upper_lid=0.85, lower_lid=0.55, pupil_size=0.60,
        iris_color=(80, 180, 120), squint=0.15,
        lower_lid_curve=1.0,
    ),

    "sad": EyeState(
        gaze_x=0.0, gaze_y=0.35,
        upper_lid=0.55, lower_lid=0.10, pupil_size=0.35,
        iris_color=(60, 80, 160), squint=0.0,
        lid_tilt=0.60, overlay='tear',
    ),

    "surprised": EyeState(
        upper_lid=1.0, lower_lid=0.0, pupil_size=0.92,
        iris_color=(80, 150, 220), squint=0.0, eyebrow=0.8,
    ),

    "angry": EyeState(
        upper_lid=0.65, lower_lid=0.30, pupil_size=0.25,
        iris_color=(200, 60, 40), squint=0.50, eyebrow=-0.8,
        lid_tilt=-0.70,
    ),

    "confused": EyeState(
        gaze_x=0.3, gaze_y=-0.2,
        upper_lid=0.75, lower_lid=0.10, pupil_size=0.48,
        iris_color=(100, 160, 80), squint=0.20,
        lid_tilt=-0.15, overlay='sweat',
    ),

    "suspicious": EyeState(
        gaze_x=0.25, gaze_y=0.0,
        upper_lid=0.55, lower_lid=0.25, pupil_size=0.30,
        iris_color=(100, 100, 60), squint=0.55,
        lid_tilt=-0.35,
    ),

    "tired": EyeState(
        gaze_x=0.0, gaze_y=0.2,
        upper_lid=0.42, lower_lid=0.10, pupil_size=0.38,
        iris_color=(60, 100, 160), squint=0.18,
        lid_tilt=0.40,
    ),

    "sleeping": EyeState(
        gaze_x=0.0, gaze_y=0.3,
        upper_lid=0.02, lower_lid=0.65, pupil_size=0.20,
        iris_color=(50, 80, 140), squint=0.0,
        overlay='zz',
    ),

    "love": EyeState(
        gaze_x=0.0, gaze_y=-0.1,
        upper_lid=0.90, lower_lid=0.25, pupil_size=0.88,
        iris_color=(220, 80, 120), squint=0.22,
        lower_lid_curve=0.5, pupil_style='heart',
    ),
}


# ---------------------------------------------------------------------------
# Behavior library
# ---------------------------------------------------------------------------

class BehaviorLibrary:

    # -- BLINKS ---------------------------------------------------------------

    @staticmethod
    def blink(speed=1.0) -> Animation:
        d = 0.07 / speed
        kf = [
            Keyframe(_closed(),  d,       EasingType.EASE_IN),
            Keyframe(_open(),    d * 1.5, EasingType.EASE_OUT),
        ]
        return Animation("blink", left_keyframes=kf, channels=CH_LIDS)

    @staticmethod
    def double_blink() -> Animation:
        d = 0.06
        kf = [
            Keyframe(_closed(), d,       EasingType.EASE_IN),
            Keyframe(_open(),   d,       EasingType.EASE_OUT, hold=0.08),
            Keyframe(_closed(), d,       EasingType.EASE_IN),
            Keyframe(_open(),   d * 1.5, EasingType.EASE_OUT),
        ]
        return Animation("double_blink", left_keyframes=kf, channels=CH_LIDS)

    @staticmethod
    def slow_blink() -> Animation:
        kf = [
            Keyframe(_closed(), 0.25, EasingType.EASE_IN),
            Keyframe(_open(),   0.35, EasingType.EASE_OUT),
        ]
        return Animation("slow_blink", left_keyframes=kf, channels=CH_LIDS)

    @staticmethod
    def wink(eye="right") -> Animation:
        wink_kf = [
            Keyframe(_closed(), 0.08, EasingType.EASE_IN),
            Keyframe(_open(),   0.15, EasingType.EASE_OUT),
        ]
        hold_kf = [
            Keyframe(_open(), 0.001),
            Keyframe(_open(), 0.001),
        ]
        if eye == "right":
            return Animation("wink_right", left_keyframes=hold_kf, right_keyframes=wink_kf,
                             channels=CH_LIDS)
        return Animation("wink_left",  left_keyframes=wink_kf, right_keyframes=hold_kf,
                         channels=CH_LIDS)

    # -- GAZE -----------------------------------------------------------------
    # Gaze behaviors are transient glances: they control only the gaze channel
    # (the active expression is preserved) and the engine eases back to the
    # base gaze when they finish. For a persistent gaze use set_base_gaze().

    @staticmethod
    def look_at(gaze_x=0.0, gaze_y=0.0, duration=0.3, hold=0.0) -> Animation:
        kf = [Keyframe(_open(gaze_x=gaze_x, gaze_y=gaze_y), duration,
                       EasingType.EASE_IN_OUT, hold=hold)]
        return Animation("look_at", left_keyframes=kf, channels=CH_GAZE)

    @staticmethod
    def look_left(amount=0.7)  -> Animation: return BehaviorLibrary.look_at(-amount, 0.0, hold=0.6)

    @staticmethod
    def look_right(amount=0.7) -> Animation: return BehaviorLibrary.look_at(amount,  0.0, hold=0.6)

    @staticmethod
    def look_up(amount=0.6)    -> Animation: return BehaviorLibrary.look_at(0.0, -amount, hold=0.6)

    @staticmethod
    def look_down(amount=0.6)  -> Animation: return BehaviorLibrary.look_at(0.0,  amount, hold=0.6)

    @staticmethod
    def look_center() -> Animation: return BehaviorLibrary.look_at(0.0, 0.0)

    @staticmethod
    def scan_horizontal() -> Animation:
        kf = [
            Keyframe(_open(gaze_x=-0.8), 0.4, EasingType.EASE_IN_OUT, hold=0.2),
            Keyframe(_open(gaze_x=0.8),  0.6, EasingType.EASE_IN_OUT, hold=0.2),
            Keyframe(_open(),            0.4, EasingType.EASE_IN_OUT),
        ]
        return Animation("scan_horizontal", left_keyframes=kf, channels=CH_GAZE)

    @staticmethod
    def saccade(gaze_x=0.7, gaze_y=0.0) -> Animation:
        kf = [Keyframe(_open(gaze_x=gaze_x, gaze_y=gaze_y), 0.04,
                       EasingType.LINEAR, hold=0.3)]
        return Animation("saccade", left_keyframes=kf, channels=CH_GAZE)

    @staticmethod
    def track_face(gaze_x=0.0, gaze_y=0.0) -> Animation:
        kf = [Keyframe(_open(gaze_x=gaze_x, gaze_y=gaze_y), 0.12, EasingType.EASE_OUT)]
        return Animation("track_face", left_keyframes=kf, channels=CH_GAZE)

    @staticmethod
    def look_away_shy() -> Animation:
        kf = [
            Keyframe(_open(gaze_x=-0.6, gaze_y=0.3), 0.3, hold=0.8),
            Keyframe(_open(), 0.4, EasingType.EASE_IN_OUT),
        ]
        return Animation("look_away_shy", left_keyframes=kf, channels=CH_GAZE)

    @staticmethod
    def glance() -> Animation:
        """Quick idle glance to a random nearby point, then back to base."""
        gx = random.uniform(-0.55, 0.55)
        gy = random.uniform(-0.30, 0.35)
        kf = [Keyframe(_open(gaze_x=gx, gaze_y=gy), 0.16, EasingType.EASE_OUT,
                       hold=random.uniform(0.4, 1.3))]
        return Animation("glance", left_keyframes=kf, channels=CH_GAZE)

    @staticmethod
    def look_around() -> Animation:
        """Casual two-point look around at idle."""
        x1 = random.uniform(0.3, 0.6) * random.choice((-1, 1))
        x2 = -x1 * random.uniform(0.4, 0.9)
        kf = [
            Keyframe(_open(gaze_x=x1, gaze_y=random.uniform(-0.2, 0.2)), 0.25,
                     EasingType.EASE_IN_OUT, hold=random.uniform(0.4, 0.9)),
            Keyframe(_open(gaze_x=x2, gaze_y=random.uniform(-0.2, 0.2)), 0.30,
                     EasingType.EASE_IN_OUT, hold=random.uniform(0.3, 0.8)),
        ]
        return Animation("look_around", left_keyframes=kf, channels=CH_GAZE)

    # -- EMOTIONS (persistent: animate TO state and stay) --------------------

    @staticmethod
    def _emotion_anim(name, entry_dur=0.22, easing=EasingType.EASE_OUT,
                      right_state=None) -> Animation:
        """Helper: build a persistent emotion animation from EMOTION_STATES."""
        st = EMOTION_STATES[name]
        lkf = [Keyframe(st, entry_dur, easing)]
        rkf = None
        if right_state is not None:
            rkf = [Keyframe(right_state, entry_dur, easing)]
        return Animation("emotion_" + name, left_keyframes=lkf, right_keyframes=rkf)

    @staticmethod
    def neutral() -> Animation:
        kf = [Keyframe(_open(), 0.35, EasingType.EASE_IN_OUT)]
        return Animation("neutral", left_keyframes=kf)

    @staticmethod
    def happy()     -> Animation: return BehaviorLibrary._emotion_anim("happy",     0.20)

    @staticmethod
    def sad()       -> Animation: return BehaviorLibrary._emotion_anim("sad",       0.50, EasingType.EASE_IN)

    @staticmethod
    def surprised() -> Animation:
        kf = [Keyframe(EMOTION_STATES["surprised"], 0.08, EasingType.ELASTIC)]
        return Animation("emotion_surprised", left_keyframes=kf)

    @staticmethod
    def angry()     -> Animation: return BehaviorLibrary._emotion_anim("angry",     0.15, EasingType.EASE_IN)

    @staticmethod
    def confused()  -> Animation:
        # Left and right eyes differ slightly for the confused asymmetry
        lkf = [Keyframe(EMOTION_STATES["confused"], 0.25)]
        rkf_st = EyeState(
            gaze_x=0.3, gaze_y=-0.2,
            upper_lid=0.60, lower_lid=0.15, pupil_size=0.50,
            iris_color=(100, 160, 80), squint=0.40,
            lid_tilt=-0.15, overlay='sweat',
        )
        rkf = [Keyframe(rkf_st, 0.25)]
        return Animation("emotion_confused", left_keyframes=lkf, right_keyframes=rkf)

    @staticmethod
    def suspicious() -> Animation:
        lkf = [Keyframe(EMOTION_STATES["suspicious"], 0.20)]
        rkf_st = EyeState(
            gaze_x=0.25, gaze_y=0.0,
            upper_lid=0.88, lower_lid=0.0, pupil_size=0.46,
            iris_color=(100, 100, 60), squint=0.08,
        )
        rkf = [Keyframe(rkf_st, 0.20)]
        return Animation("emotion_suspicious", left_keyframes=lkf, right_keyframes=rkf)

    @staticmethod
    def tired()     -> Animation: return BehaviorLibrary._emotion_anim("tired",     0.80, EasingType.EASE_IN)

    @staticmethod
    def love()      -> Animation:
        kf = [Keyframe(EMOTION_STATES["love"], 0.30, EasingType.BOUNCE)]
        return Animation("emotion_love", left_keyframes=kf)

    # -- SLEEP / WAKE --------------------------------------------------------

    @staticmethod
    def fall_asleep() -> Animation:
        kf = [
            Keyframe(HALF_CLOSED,              1.5, EasingType.EASE_IN, hold=0.5),
            Keyframe(EMOTION_STATES["sleeping"], 1.0, EasingType.EASE_IN),
        ]
        return Animation("fall_asleep", left_keyframes=kf)

    @staticmethod
    def wake_up() -> Animation:
        kf = [
            Keyframe(HALF_CLOSED, 0.8, EasingType.EASE_OUT, hold=0.3),
            Keyframe(_open(),     0.6, EasingType.ELASTIC),
        ]
        return Animation("wake_up", left_keyframes=kf)

    @staticmethod
    def sleeping_loop() -> Animation:
        sl1 = EMOTION_STATES["sleeping"]
        sl2 = EyeState(
            gaze_x=0.0, gaze_y=0.3,
            upper_lid=0.0, lower_lid=0.70, pupil_size=0.20,
            iris_color=(50, 80, 140),
            overlay='zz',
        )
        kf = [
            Keyframe(sl1, 1.8, EasingType.EASE_IN_OUT),
            Keyframe(sl2, 2.2, EasingType.EASE_IN_OUT),
        ]
        return Animation("sleeping_loop", left_keyframes=kf, loop=True)

    # -- ATTENTION / MISC ----------------------------------------------------

    @staticmethod
    def notice(gaze_x=0.5, gaze_y=0.0) -> Animation:
        kf = [
            Keyframe(_open(gaze_x=gaze_x, gaze_y=gaze_y, pupil=0.7),
                     0.08, EasingType.LINEAR, hold=0.4),
            Keyframe(_open(), 0.3, EasingType.EASE_IN_OUT),
        ]
        return Animation("notice", left_keyframes=kf, channels=CH_GAZE_PUPIL)

    @staticmethod
    def thinking() -> Animation:
        kf = [
            Keyframe(_thinking(gaze_x=0.5, gaze_y=-0.4), 0.25, hold=0.8),
            Keyframe(_thinking(gaze_x=0.2, gaze_y=-0.3), 0.3,  hold=0.4),
            Keyframe(_thinking(), 0.4),
        ]
        return Animation("thinking", left_keyframes=kf, channels=CH_THINK)

    @staticmethod
    def thinking_loop() -> Animation:
        """Looping 'pondering' expression while the robot waits for the LLM:
        narrowed lids, raised brow, contracted pupil, cyan iris and gaze
        drifting up-sideways -> reads clearly as 'thinking', not an emotion.
        Cancelled via engine.cancel_if('thinking_loop') or by the next anim."""
        kf = [
            Keyframe(_thinking(gaze_x=0.5,  gaze_y=-0.4),  0.30, hold=0.7),
            Keyframe(_thinking(gaze_x=0.25, gaze_y=-0.45), 0.35, hold=0.5),
            Keyframe(_thinking(gaze_x=0.55, gaze_y=-0.25), 0.35, hold=0.6),
        ]
        return Animation("thinking_loop", left_keyframes=kf, channels=CH_THINK,
                         loop=True)

    @staticmethod
    def listening() -> Animation:
        """Attention perk: eyes open wide and pupils dilate briefly, signalling
        'I heard you' right after speech is detected."""
        st = EyeState(upper_lid=1.0, lower_lid=0.0, pupil_size=0.72)
        kf = [Keyframe(st, 0.15, EasingType.EASE_OUT, hold=1.4)]
        return Animation("listening", left_keyframes=kf,
                         channels=frozenset({'lids', 'pupil'}))

    @staticmethod
    def speaking() -> Animation:
        """Subtle lively eye motion while the robot's voice is playing.
        Loops until cancelled via engine.cancel_if('speaking')."""
        kf = [
            Keyframe(_open(gaze_y=-0.05, pupil=0.55),               0.30, EasingType.EASE_IN_OUT),
            Keyframe(_open(gaze_x=0.05,  gaze_y=0.03, pupil=0.50),  0.35, EasingType.EASE_IN_OUT),
            Keyframe(_open(gaze_x=-0.05, gaze_y=-0.02, pupil=0.58), 0.32, EasingType.EASE_IN_OUT),
            Keyframe(_open(gaze_y=0.04,  pupil=0.52),               0.36, EasingType.EASE_IN_OUT),
        ]
        return Animation("speaking", left_keyframes=kf, channels=CH_GAZE_PUPIL,
                         loop=True)

    @staticmethod
    def dizzy() -> Animation:
        # Spiral pupils while the gaze whirls around, back to normal at the end
        pts = [(0.8, 0.0), (0.0, -0.8), (-0.8, 0.0),
               (0.0, 0.8), (0.8, 0.0), (0.0, -0.8), (-0.8, 0.0)]
        def _spin(x, y):
            st = _open(gaze_x=x, gaze_y=y)
            st.pupil_style = 'spiral'
            return st
        kf  = [Keyframe(_spin(x, y), 0.15) for x, y in pts]
        kf += [Keyframe(_open(), 0.3, EasingType.EASE_OUT)]
        return Animation("dizzy", left_keyframes=kf, channels=CH_GAZE_PUPIL)

    @staticmethod
    def roll_eyes() -> Animation:
        pts = [(0.0, -0.8), (0.8, -0.4), (0.8, 0.4),
               (0.0, 0.6), (-0.8, 0.0), (0.0, 0.0)]
        kf  = [Keyframe(_open(gaze_x=x, gaze_y=y), 0.2) for x, y in pts]
        return Animation("roll_eyes", left_keyframes=kf, channels=CH_GAZE)

    @staticmethod
    def pupil_dilate(amount=1.0) -> Animation:
        kf = [
            Keyframe(_open(pupil=amount), 0.3, EasingType.EASE_OUT, hold=1.0),
            Keyframe(_open(), 0.5, EasingType.EASE_IN_OUT),
        ]
        return Animation("pupil_dilate", left_keyframes=kf, channels=CH_PUPIL)

    @staticmethod
    def random_behavior() -> Animation:
        """
        Idle movement: mostly small glances, occasionally a longer look.
        Blinking is NOT included here -- the engine auto-blinks on its own.
        """
        choices = [
            (BehaviorLibrary.glance,        5),
            (BehaviorLibrary.look_around,   2),
            (BehaviorLibrary.thinking,      1),
            (BehaviorLibrary.look_away_shy, 1),
        ]
        factories, weights = zip(*choices)
        return random.choices(factories, weights=weights)[0]()


# ---------------------------------------------------------------------------
# Public maps
# ---------------------------------------------------------------------------

BEHAVIOR_MAP = {
    "blink":         BehaviorLibrary.blink,
    "double_blink":  BehaviorLibrary.double_blink,
    "slow_blink":    BehaviorLibrary.slow_blink,
    "wink_right":    lambda: BehaviorLibrary.wink("right"),
    "wink_left":     lambda: BehaviorLibrary.wink("left"),
    "look_left":     BehaviorLibrary.look_left,
    "look_right":    BehaviorLibrary.look_right,
    "look_up":       BehaviorLibrary.look_up,
    "look_down":     BehaviorLibrary.look_down,
    "look_center":   BehaviorLibrary.look_center,
    "glance":        BehaviorLibrary.glance,
    "look_around":   BehaviorLibrary.look_around,
    "scan":          BehaviorLibrary.scan_horizontal,
    "thinking":      BehaviorLibrary.thinking,
    "thinking_loop": BehaviorLibrary.thinking_loop,
    "listening":     BehaviorLibrary.listening,
    "speaking":      BehaviorLibrary.speaking,
    "roll_eyes":     BehaviorLibrary.roll_eyes,
    "dizzy":         BehaviorLibrary.dizzy,
    "look_away":     BehaviorLibrary.look_away_shy,
    "neutral":       BehaviorLibrary.neutral,
    "happy":         BehaviorLibrary.happy,
    "sad":           BehaviorLibrary.sad,
    "surprised":     BehaviorLibrary.surprised,
    "angry":         BehaviorLibrary.angry,
    "suspicious":    BehaviorLibrary.suspicious,
    "tired":         BehaviorLibrary.tired,
    "love":          BehaviorLibrary.love,
    "confused":      BehaviorLibrary.confused,
    "fall_asleep":   BehaviorLibrary.fall_asleep,
    "wake_up":       BehaviorLibrary.wake_up,
    "sleeping":      BehaviorLibrary.sleeping_loop,
    "dilate":        BehaviorLibrary.pupil_dilate,
    "notice":        BehaviorLibrary.notice,
    "random":        BehaviorLibrary.random_behavior,
    "saccade_right": lambda: BehaviorLibrary.saccade(0.7,  0.0),
    "saccade_left":  lambda: BehaviorLibrary.saccade(-0.7, 0.0),
}
