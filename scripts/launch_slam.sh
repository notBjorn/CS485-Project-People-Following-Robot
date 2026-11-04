#!/usr/bin/env bash
# Simulation + SLAM, for building a map. Undock, drive around with launch_teleop.sh,
# then save it with save_map.sh. Extra args pass through, e.g. world:=maze
source "$(dirname "$0")/env.sh"
source "$(dirname "$0")/sim_start.sh"
LOG="$PROJECT_ROOT/log/slam_launch.log"
stop_sim_on_exit
echo "Simulation + SLAM starting..."
start_sim "$LOG" slam:=true "$@" || exit 1
echo "Undock before driving (see RUN_DIRECTIONS.txt), and turn slowly: fast turns rotate the map."
follow_sim "$LOG"
