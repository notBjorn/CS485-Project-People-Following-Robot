#!/usr/bin/env bash
# Demo 1: the robot follows a person with its camera (YOLO + depth + tracker), through
# the safety guard, in simple_colored_warehouse. The person walks once round the
# coloured box in each of the four rooms (134 m, about 8 minutes) and stops back at
# the start; the robot follows about 1-1.5 m behind.
#
#   ./scripts/demo_follow.sh                 # clean camera
#   ./scripts/demo_follow.sh noise:=true     # real-world camera errors
#
# Opens Gazebo and a camera window (the robot's view with YOLO's boxes). Runs until
# Ctrl+C, which stops everything. Logs: log/demo_follow/.
# Builds the workspace first (./scripts/build.sh), and sets up YOLO the first time
# (./scripts/setup_yolo.sh), so there are no separate setup steps.
source "$(dirname "$0")/demo_common.sh"
NOISE="${1:-noise:=false}"
demo_setup demo_follow
demo_start_sim
demo_start_follower "$NOISE"
demo_camera_view

echo "The person starts walking round the four rooms."
"$SCRIPTS/launch_human_waypoints.sh" warehouse_boxes > "$LOGS/walk.log" 2>&1 &
DEMO_PIDS+=($!)
demo_watch "$LOGS/walk.log"
