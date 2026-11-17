#!/usr/bin/env bash
# Simulation + SLAM, for building a map. Drive around with launch_teleop.sh (TurtleBot 4:
# undock first), then save it with save_map.sh. Extra args pass through, e.g. world:=maze
# or robot:=turtlebot4
source "$(dirname "$0")/env.sh"
source "$(dirname "$0")/sim_start.sh"
LOG="$PROJECT_ROOT/log/slam_launch.log"
stop_sim_on_exit
echo "Simulation + SLAM starting..."
start_sim "$LOG" slam:=true "$@" || exit 1
echo "Turn slowly: fast turns rotate the map. TurtleBot 4: undock before driving (see RUN_DIRECTIONS.txt)."
follow_sim "$LOG"
