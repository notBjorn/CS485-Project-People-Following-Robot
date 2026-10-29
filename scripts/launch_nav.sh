#!/usr/bin/env bash
# Simulation + AMCL localization + Nav2 on the saved map for the world.
# Extra args pass through, e.g. world:=warehouse or map:=/path/to/map.yaml
source "$(dirname "$0")/env.sh"
ros2 launch people_follower sim.launch.py localization:=true nav2:=true "$@"
