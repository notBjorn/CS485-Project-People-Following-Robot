#!/usr/bin/env bash
# Open the person_world world: 25 m x 25 m walled square with one person to follow.
# Gazebo + TurtleBot 4 + RViz. Extra args pass through, e.g. rviz:=false x:=1.0 y:=-2.0
exec "$(dirname "$0")/launch_sim.sh" world:=person_world "$@"
