#!/usr/bin/env bash
# Open the box_room world: Empty 10 m x 10 m walled room (the default world).
# Gazebo + TurtleBot 4 + RViz. Extra args pass through, e.g. rviz:=false x:=1.0 y:=-2.0
exec "$(dirname "$0")/launch_sim.sh" world:=box_room "$@"
