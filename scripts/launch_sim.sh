#!/usr/bin/env bash
# Gazebo + robot + RViz. The robot is TIAGo unless you pass robot:=turtlebot4.
# Extra args pass through, e.g.: ./launch_sim.sh world:=simple_colored_warehouse
#                                ./launch_sim.sh robot:=turtlebot4 world:=warehouse model:=lite
source "$(dirname "$0")/env.sh"
source "$(dirname "$0")/sim_start.sh"
LOG="$PROJECT_ROOT/log/sim_launch.log"
stop_sim_on_exit
echo "Simulation starting..."
start_sim "$LOG" "$@" || exit 1
follow_sim "$LOG"
