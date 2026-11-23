#!/usr/bin/env bash
# Drive the simulated robot (TIAGo or TurtleBot 4) from the keyboard (run in its own terminal).
# Both take stamped Twist messages on /cmd_vel (for TIAGo, tiago_base_relay forwards
# them to /mobile_base_controller/cmd_vel).
source "$(dirname "$0")/env.sh"
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
