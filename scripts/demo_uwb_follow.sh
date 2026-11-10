#!/usr/bin/env bash
# Demo 1 (UWB): the robot follows a person using UWB ranging - four anchors on the
# robot, a tag on the person's back - in simple_colored_warehouse. The person walks once
# round the coloured box in each of the four rooms (134 m, about 8 minutes) and stops
# back at the start; the robot follows their path.
#
#   ./scripts/demo_uwb_follow.sh
#
# Opens Gazebo. The person stands still for the first few seconds while the localizer
# calibrates the anchors, then starts walking. Runs until Ctrl+C, which stops
# everything. Logs: log/demo_uwb_follow/.
# Needs a built workspace (./scripts/build.sh).
source "$(dirname "$0")/demo_uwb_common.sh"
demo_setup demo_uwb_follow
demo_start_sim
demo_start_follower

echo "The person starts walking round the four rooms."
"$SCRIPTS/launch_human_waypoints.sh" warehouse_boxes > "$LOGS/walk.log" 2>&1 &
DEMO_PIDS+=($!)
demo_watch "$LOGS/walk.log"
