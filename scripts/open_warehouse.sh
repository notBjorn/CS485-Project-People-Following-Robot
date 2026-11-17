#!/usr/bin/env bash
# Open the warehouse world: TurtleBot 4's stock warehouse world (downloads models on first run).
# Gazebo + robot (TIAGo, or robot:=turtlebot4) + RViz. Extra args pass through,
# e.g. rviz:=false x:=1.0 y:=-2.0 robot:=turtlebot4
exec "$(dirname "$0")/launch_sim.sh" world:=warehouse "$@"
