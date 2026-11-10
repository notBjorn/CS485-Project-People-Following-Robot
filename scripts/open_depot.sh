#!/usr/bin/env bash
# Open the depot world: TurtleBot 4's stock depot world (downloads models on first run).
# Gazebo + TurtleBot 4 + RViz. Extra args pass through, e.g. rviz:=false x:=1.0 y:=-2.0
exec "$(dirname "$0")/launch_sim.sh" world:=depot "$@"
