#!/usr/bin/env bash
# Run the automated navigation node (needs launch_nav.sh running).
# Extra args are ROS parameters, e.g.: ./scripts/launch_nav_node.sh -p goal_x:=3.0 -p goal_y:=-2.0
source "$(dirname "$0")/env.sh"
ros2 run turtlebot4_waypoint_auto_nav nav_automation_node --ros-args -p use_sim_time:=true "$@"
