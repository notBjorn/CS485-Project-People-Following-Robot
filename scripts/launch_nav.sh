#!/usr/bin/env bash
# Simulation + AMCL localization + Nav2 on the saved map for the world.
# Extra args pass through, e.g. world:=warehouse, robot:=turtlebot4 or map:=/path/to/map.yaml
#
# Also sets the initial pose to the spawn pose (x:=, y:=, yaw:=, default 0 0 0): Nav2 can't
# finish starting without one and gives up after about a minute. To correct it later, use
# set_initial_pose.sh or RViz's "2D Pose Estimate".
source "$(dirname "$0")/env.sh"
source "$(dirname "$0")/sim_start.sh"

X=0; Y=0; YAW=0
for arg in "$@"; do
    case "$arg" in
        x:=*) X=${arg#x:=} ;;
        y:=*) Y=${arg#y:=} ;;
        yaw:=*) YAW=${arg#yaw:=} ;;
    esac
done
DEG=$(awk -v r="$YAW" 'BEGIN { printf "%f", r * 180 / 3.14159265 }')

set_pose_when_ready() {
    for _ in $(seq 1 45); do
        if timeout -k 2 5 ros2 lifecycle get /amcl 2>/dev/null | grep -q '^active'; then
            "$PROJECT_ROOT/scripts/set_initial_pose.sh" "$X" "$Y" "$DEG" > /dev/null \
                && echo "Initial pose set to x=$X y=$Y yaw=$YAW" \
                || echo "Couldn't set the initial pose - use set_initial_pose.sh or RViz" >&2
            return
        fi
        sleep 1
    done
    echo "AMCL didn't start, so the initial pose wasn't set" >&2
}

LOG="$PROJECT_ROOT/log/nav_launch.log"
stop_sim_on_exit
echo "Simulation + localization + Nav2 starting..."
start_sim "$LOG" localization:=true nav2:=true "$@" || exit 1
set_pose_when_ready &
follow_sim "$LOG"
