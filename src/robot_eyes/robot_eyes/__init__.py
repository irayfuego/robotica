from .eye_renderer import EyeRenderer, EyeState
from .animation_engine import AnimationEngine, Animation, Keyframe, EasingType
from .behaviors import BehaviorLibrary
from .display_driver import GC9A01, DualDisplayController

__version__ = "1.0.0"
__all__ = [
    "EyeRenderer", "EyeState",
    "AnimationEngine", "Animation", "Keyframe", "EasingType",
    "BehaviorLibrary",
    "GC9A01", "DualDisplayController",
]
