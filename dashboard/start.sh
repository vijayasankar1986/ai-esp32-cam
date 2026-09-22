#!/usr/bin/env bash
set -e
cd /home/bbt/ai-esp32cam
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
exec /usr/bin/python3 dashboard/server.py
