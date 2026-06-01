#!/usr/bin/env python3
"""
Example ROS clients for robot_eyes.

This file contains ready-to-use example scripts showing how to:
  1. Play a specific behavior
  2. Control gaze from a face detector
  3. Run a demo sequence
  4. Integrate with an emotion classifier

Run any example:
  python examples.py demo
  python examples.py face_track
  python examples.py emotion_demo
"""

import sys
import time
import math

try:
    import rospy
    from std_msgs.msg import String, Float32
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("ROS not available. These examples require a running ROS master.")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Example 1: Play behaviors by publishing to /robot_eyes/behavior
# ---------------------------------------------------------------------------
def example_behavior_sequence():
    """Play a scripted sequence of behaviors."""
    rospy.init_node("robot_eyes_example_sequence", anonymous=True)
    pub = rospy.Publisher("/robot_eyes/behavior", String, queue_size=1)
    time.sleep(0.5)  # wait for connection

    sequence = [
        ("wake_up",    2.0),
        ("surprised",  2.0),
        ("happy",      2.5),
        ("blink",      0.5),
        ("look_left",  1.0),
        ("look_right", 1.0),
        ("look_center",0.5),
        ("thinking",   2.0),
        ("wink_right", 1.0),
        ("double_blink",0.5),
        ("tired",      3.0),
        ("fall_asleep",3.0),
        ("wake_up",    2.0),
        ("happy",      2.0),
    ]

    rospy.loginfo("Playing behavior sequence...")
    for behavior, delay in sequence:
        if rospy.is_shutdown():
            break
        rospy.loginfo(f"  -> {behavior}")
        pub.publish(String(data=behavior))
        time.sleep(delay)

    rospy.loginfo("Sequence complete.")


# ---------------------------------------------------------------------------
# Example 2: Face tracking (simulated circular movement)
# ---------------------------------------------------------------------------
def example_face_tracking():
    """
    Simulate a face moving in a circle.
    In production, replace with your face detection node output.

    Your face detector should publish:
      /robot_eyes/face_position  [geometry_msgs/Point]
        x: normalized horizontal  [-1 left, +1 right]
        y: normalized vertical    [-1 up,   +1 down]
        z: distance (unused by eyes node)
    """
    rospy.init_node("robot_eyes_face_track_example", anonymous=True)
    pub_face = rospy.Publisher("/robot_eyes/face_position", Point, queue_size=1)
    pub_beh  = rospy.Publisher("/robot_eyes/behavior", String, queue_size=1)
    time.sleep(0.5)

    # Simulate "face detected"
    pub_beh.publish(String(data="surprised"))
    time.sleep(1.0)

    rospy.loginfo("Simulating face tracking (circular path)...")
    rate = rospy.Rate(20)  # 20 Hz tracking
    t = 0.0
    while not rospy.is_shutdown() and t < 12.0:
        # Slow circular movement
        x = 0.5 * math.sin(t * 0.8)
        y = 0.3 * math.cos(t * 0.6)
        pub_face.publish(Point(x=x, y=y, z=1.0))
        t += 0.05
        rate.sleep()

    # Return to center
    pub_face.publish(Point(x=0, y=0, z=1.0))
    rospy.loginfo("Face tracking example complete.")


# ---------------------------------------------------------------------------
# Example 3: Emotion demo from an external classifier
# ---------------------------------------------------------------------------
def example_emotion_demo():
    """
    Demonstrate emotion transitions.
    In production, subscribe to your emotion classifier and
    publish to /robot_eyes/emotion.
    """
    rospy.init_node("robot_eyes_emotion_demo", anonymous=True)
    pub_emotion = rospy.Publisher("/robot_eyes/emotion",   String,    queue_size=1)
    pub_pupil   = rospy.Publisher("/robot_eyes/pupil_size", Float32,  queue_size=1)
    pub_color   = rospy.Publisher("/robot_eyes/iris_color", ColorRGBA, queue_size=1)
    time.sleep(0.5)

    def set_emotion(name, pupil=0.5, r=60, g=120, b=200, hold=2.0):
        pub_emotion.publish(String(data=name))
        pub_pupil.publish(Float32(data=pupil))
        pub_color.publish(ColorRGBA(r=r/255, g=g/255, b=b/255, a=1.0))
        rospy.loginfo(f"  Emotion: {name}")
        time.sleep(hold)

    emotions = [
        # (name,       pupil, R,   G,   B,   hold)
        ("neutral",    0.5,   60,  120, 200, 2.0),
        ("happy",      0.65,  80,  180, 80,  2.5),
        ("surprised",  0.9,   100, 160, 220, 2.0),
        ("angry",      0.2,   200, 50,  40,  2.5),
        ("sad",        0.35,  50,  70,  160, 2.5),
        ("love",       0.85,  220, 80,  130, 2.5),
        ("confused",   0.5,   100, 160, 80,  2.5),
        ("suspicious", 0.3,   100, 100, 60,  2.5),
        ("tired",      0.4,   60,  100, 160, 3.0),
        ("neutral",    0.5,   60,  120, 200, 1.5),
    ]

    rospy.loginfo("Playing emotion demo...")
    for args in emotions:
        if rospy.is_shutdown():
            break
        set_emotion(*args)

    rospy.loginfo("Emotion demo complete.")


# ---------------------------------------------------------------------------
# Example 4: Gaze control with keyboard
# ---------------------------------------------------------------------------
def example_gaze_keyboard():
    """
    Control gaze with WASD keys.
    Requires: pip install keyboard (run as root or with sudo)
    """
    try:
        import keyboard
    except ImportError:
        print("Install 'keyboard': pip install keyboard")
        sys.exit(1)

    rospy.init_node("robot_eyes_gaze_keyboard", anonymous=True)
    pub_gaze = rospy.Publisher("/robot_eyes/gaze", Point, queue_size=1)
    pub_beh  = rospy.Publisher("/robot_eyes/behavior", String, queue_size=1)
    time.sleep(0.5)

    print("WASD = gaze, B = blink, W2 = wink right, Q = quit")
    gx, gy = 0.0, 0.0
    speed = 0.05
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        changed = False
        if keyboard.is_pressed("a"):
            gx = max(-0.9, gx - speed); changed = True
        if keyboard.is_pressed("d"):
            gx = min(0.9,  gx + speed); changed = True
        if keyboard.is_pressed("w"):
            gy = max(-0.9, gy - speed); changed = True
        if keyboard.is_pressed("s"):
            gy = min(0.9,  gy + speed); changed = True
        if keyboard.is_pressed("b"):
            pub_beh.publish(String(data="blink"))
            time.sleep(0.3)
        if keyboard.is_pressed("e"):
            pub_beh.publish(String(data="wink_right"))
            time.sleep(0.5)
        if keyboard.is_pressed("q"):
            break
        if changed:
            pub_gaze.publish(Point(x=gx, y=gy, z=0))
        # Drift back to center
        gx *= 0.98; gy *= 0.98
        rate.sleep()


# ---------------------------------------------------------------------------
# Example 5: Integration with OpenCV face detector
# ---------------------------------------------------------------------------
FACE_TRACKING_CODE = '''
#!/usr/bin/env python3
"""
OpenCV face detection + robot_eyes integration.
Install: pip install opencv-python
Run alongside the robot_eyes node.
"""
import cv2
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String

rospy.init_node("face_tracker")
pub_face = rospy.Publisher("/robot_eyes/face_position", Point, queue_size=1)
pub_beh  = rospy.Publisher("/robot_eyes/behavior", String, queue_size=1)

cap = cv2.VideoCapture(0)
detector = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
cam_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
cam_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

face_visible_prev = False
rate = rospy.Rate(20)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        continue
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = detector.detectMultiScale(gray, 1.1, 4)

    if len(faces) > 0:
        # Track the largest face
        x, y, w, h = max(faces, key=lambda f: f[2]*f[3])
        cx = x + w / 2
        cy = y + h / 2
        # Normalize to [-1, 1]
        nx = (cx - cam_w / 2) / (cam_w / 2)
        ny = (cy - cam_h / 2) / (cam_h / 2)
        pub_face.publish(Point(x=nx, y=ny, z=1.0))

        if not face_visible_prev:
            pub_beh.publish(String(data="notice"))
        face_visible_prev = True
    else:
        if face_visible_prev:
            pub_beh.publish(String(data="look_center"))
        face_visible_prev = False

    rate.sleep()

cap.release()
'''


if __name__ == "__main__":
    examples = {
        "demo":         example_behavior_sequence,
        "face_track":   example_face_tracking,
        "emotion_demo": example_emotion_demo,
        "keyboard":     example_gaze_keyboard,
    }

    if len(sys.argv) < 2 or sys.argv[1] not in examples:
        print("Usage: python examples.py <example>")
        print(f"Examples: {list(examples.keys())}")
        print()
        print("Face detector integration code:")
        print(FACE_TRACKING_CODE)
        sys.exit(1)

    examples[sys.argv[1]]()
