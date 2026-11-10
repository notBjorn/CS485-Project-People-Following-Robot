#!/usr/bin/env bash
# Open the simple_colored_warehouse world: 30 m x 24 m warehouse: four rooms with coloured boxes, and one person.
# Gazebo + TurtleBot 4 + RViz. Extra args pass through, e.g. rviz:=false x:=1.0 y:=-2.0
exec "$(dirname "$0")/launch_sim.sh" world:=simple_colored_warehouse "$@"
