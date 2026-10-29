#!/usr/bin/env bash
# One command for automated navigation: starts the sim + localization + Nav2,
# waits until Nav2 is active, then runs nav_automation_node. Ctrl+C stops everything.
# Usage: ./scripts/nav_demo.sh [sim args] [-- node args]
#   e.g. ./scripts/nav_demo.sh world:=warehouse -- -p goal_x:=3.0 -p goal_y:=-2.0
source "$(dirname "$0")/env.sh"
SIM_ARGS=(); NODE_ARGS=()
while [ $# -gt 0 ]; do
    if [ "$1" = "--" ]; then shift; NODE_ARGS=("$@"); break; fi
    SIM_ARGS+=("$1"); shift
done

LOG="$PROJECT_ROOT/log/nav_demo_launch.log"
mkdir -p "$PROJECT_ROOT/log"
ros2 launch people_follower sim.launch.py localization:=true nav2:=true "${SIM_ARGS[@]}" > "$LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
    trap - INT TERM EXIT
    echo "Stopping simulation..."
    # Background jobs ignore Ctrl+C's SIGINT, so stop the launch with SIGTERM
    kill -TERM "$LAUNCH_PID" 2>/dev/null
    for _ in $(seq 1 15); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
    kill -0 "$LAUNCH_PID" 2>/dev/null && "$PROJECT_ROOT/scripts/kill_sim.sh"
}
trap cleanup EXIT
trap "cleanup; exit 130" INT TERM

echo "Simulation starting (output in $LOG). Waiting for Nav2..."
TIMEOUT=300   # the sim is slow on some machines
for ((t = 0; t < TIMEOUT; t += 5)); do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "Launch exited early - last lines of $LOG:"; tail -20 "$LOG"; exit 1
    fi
    if timeout 5 ros2 lifecycle get /amcl 2>/dev/null | grep -q '^active' &&
       timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q '^active'; then
        break
    fi
    sleep 5
done
if [ "$t" -ge "$TIMEOUT" ]; then echo "Nav2 did not become active within ${TIMEOUT} s"; exit 1; fi

echo "Nav2 active - starting nav_automation_node"
ros2 run turtlebot4_waypoint_auto_nav nav_automation_node --ros-args -p use_sim_time:=true "${NODE_ARGS[@]}"
