#!/usr/bin/env bash
# Simulation + SLAM, for building a map. Drive around with launch_teleop.sh,
# then save it with save_map.sh. Extra args pass through, e.g. world:=maze
source "$(dirname "$0")/env.sh"
ros2 launch people_follower sim.launch.py slam:=true "$@"
