#!/usr/bin/env python3
"""
launch_eyes.py -- arranque directo desde systemd
Lanza robot_eyes_node, huskylens_gaze_bridge, huskylens_tts_node y
voice_command_node en el mismo proceso usando MultiThreadedExecutor.
Los parametros se cargan desde config/robot_eyes_params.yaml via --params-file.
"""
import sys
sys.path.insert(0, "/home/mimavi/robotica_ws/install/robot_eyes/lib/python3.12/site-packages")

import rclpy
from rclpy.executors import MultiThreadedExecutor
from robot_eyes.ros_node import RobotEyesNode
from robot_eyes.huskylens_gaze_bridge import HuskyLensGazeBridge
from robot_eyes.huskylens_tts_node import HuskyLensTtsNode
from robot_eyes.voice_command_node import VoiceCommandNode

PARAMS_FILE = "/home/mimavi/robotica_ws/src/robot_eyes/config/robot_eyes_params.yaml"


def main():
    rclpy.init(args=["--ros-args", "--params-file", PARAMS_FILE])

    eyes_node   = RobotEyesNode()
    bridge_node = HuskyLensGazeBridge()
    tts_node    = HuskyLensTtsNode()
    voice_node  = VoiceCommandNode()

    executor = MultiThreadedExecutor()
    executor.add_node(eyes_node)
    executor.add_node(bridge_node)
    executor.add_node(tts_node)
    executor.add_node(voice_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        eyes_node.shutdown()
        tts_node.shutdown()
        voice_node.shutdown()
        eyes_node.destroy_node()
        bridge_node.destroy_node()
        tts_node.destroy_node()
        voice_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
