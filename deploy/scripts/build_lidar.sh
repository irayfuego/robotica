#!/bin/bash
source /opt/ros/jazzy/setup.bash
cd /home/mimavi/robotica_ws
MAKEFLAGS=-j1 colcon build --packages-select m1ct_d2 --parallel-workers 1   >> /tmp/build_lidar.log 2>&1
if [ $? -eq 0 ]; then
  echo 'BUILD_OK' >> /tmp/build_lidar.log
  systemctl start robot-bringup
else
  echo 'BUILD_FAILED' >> /tmp/build_lidar.log
fi
