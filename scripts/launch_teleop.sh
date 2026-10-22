#!/usr/bin/env bash
# Drive the simulated TurtleBot 4 from the keyboard (run in its own terminal).
# The TB4 base expects stamped Twist messages.
source "$(dirname "$0")/env.sh"
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
