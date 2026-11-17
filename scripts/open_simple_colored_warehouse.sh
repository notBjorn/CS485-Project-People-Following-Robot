#!/usr/bin/env bash
# Open the simple_colored_warehouse world: 30 m x 24 m warehouse with four rooms and
# coloured boxes, and one person wearing a UWB tag (either robot carries the anchors).
# Gazebo + robot (TIAGo, or robot:=turtlebot4) + RViz. Extra args pass through,
# e.g. rviz:=false x:=1.0 y:=-2.0 robot:=turtlebot4
exec "$(dirname "$0")/launch_sim.sh" world:=simple_colored_warehouse "$@"
