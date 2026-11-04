#!/usr/bin/env bash
# One command for automated navigation: starts the sim + localization + Nav2,
# waits until AMCL is active, then runs nav_automation_node. Ctrl+C stops everything.
# Don't wait for all of Nav2 here: it can't finish starting until the node sets the initial pose.
# Usage: ./scripts/nav_demo.sh [sim args] [-- node args]
#   e.g. ./scripts/nav_demo.sh world:=warehouse -- -p goal_x:=3.0 -p goal_y:=-2.0
source "$(dirname "$0")/env.sh"
source "$(dirname "$0")/sim_start.sh"
SIM_ARGS=(); NODE_ARGS=()
while [ $# -gt 0 ]; do
    if [ "$1" = "--" ]; then shift; NODE_ARGS=("$@"); break; fi
    SIM_ARGS+=("$1"); shift
done

LOG="$PROJECT_ROOT/log/nav_demo_launch.log"
stop_sim_on_exit
echo "Simulation starting (output in $LOG)."
start_sim "$LOG" localization:=true nav2:=true "${SIM_ARGS[@]}" || exit 1

echo "Waiting for AMCL..."
TIMEOUT=300   # the sim is slow on some machines
for ((t = 0; t < TIMEOUT; t += 5)); do
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "Launch exited early - last lines of $LOG:"; tail -20 "$LOG"; exit 1
    fi
    if timeout -k 2 5 ros2 lifecycle get /amcl 2>/dev/null | grep -q '^active'; then
        break
    fi
    sleep 5
done
if [ "$t" -ge "$TIMEOUT" ]; then echo "AMCL did not become active within ${TIMEOUT} s"; exit 1; fi

echo "AMCL active - starting nav_automation_node"
ros2 run turtlebot4_waypoint_auto_nav nav_automation_node --ros-args -p use_sim_time:=true "${NODE_ARGS[@]}"
