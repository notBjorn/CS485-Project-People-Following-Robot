#!/usr/bin/env bash
# Gazebo + TurtleBot 4 + RViz.
# Extra args pass through, e.g.: ./launch_sim.sh world:=warehouse model:=lite
source "$(dirname "$0")/env.sh"
ros2 launch people_follower sim.launch.py "$@"
