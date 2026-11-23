#!/usr/bin/env bash
# Demo 3 (UWB): the robot follows a person through every room of turtlebot4_house, the
# detailed 10 m x 8 m house (living room, dining room, bedroom, bathroom). Four anchors
# on the robot, a tag on the person's back. The person walks one lap (44 m, about
# 3 minutes): through the bedroom door and round the bedroom, through the arch into the
# dining room and round the table, through the bathroom door and round the bathroom,
# then back to the living room. The robot follows their path through the 0.9 m doors.
#
#   ./scripts/demo_uwb_house.sh                      # TIAGo
#   ./scripts/demo_uwb_house.sh robot:=turtlebot4
#
# Opens Gazebo. The person stands still for the first few seconds while the localizer
# calibrates the anchors, then starts walking. Runs until Ctrl+C, which stops
# everything. Logs: log/demo_uwb_house/. No map is needed: following uses only UWB,
# odometry and the LiDAR.
# Builds the workspace first (./scripts/build.sh), so there is no separate build step.
source "$(dirname "$0")/demo_uwb_common.sh"
demo_setup demo_uwb_house
# Robot in the living room, 2.2 m behind the person (who starts at (0, 1) facing north).
demo_start_sim turtlebot4_house 0.0 -1.2 1.5708
demo_start_follower

echo "The person starts walking through the four rooms."
"$SCRIPTS/launch_human_waypoints.sh" house > "$LOGS/walk.log" 2>&1 &
DEMO_PIDS+=($!)
demo_watch "$LOGS/walk.log"
