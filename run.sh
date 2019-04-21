#!/bin/bash
echo "[RUN] Sourcing..."
source ~/ros2_ws/install/local_setup.bash
source ./install/local_setup.bash
echo "[RUN] Running using ROS2..."
ros2 run robug $1