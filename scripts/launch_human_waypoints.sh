#!/usr/bin/env bash
# Walk the person through waypoints:
#   ./scripts/launch_human_waypoints.sh             person_world: config/human_waypoints.yaml
#   ./scripts/launch_human_waypoints.sh warehouse   simple_colored_warehouse, square loop:
#                                                   config/human_waypoints_warehouse.yaml
#   ./scripts/launch_human_waypoints.sh warehouse_boxes   simple_colored_warehouse, around
#                                       each coloured box: human_waypoints_warehouse_boxes.yaml
# Start the matching world first (launch_sim.sh world:=...).
# Don't run it together with launch_human_teleop.sh: both drive the person.
# Extra args pass through, e.g.: ./scripts/launch_human_waypoints.sh params_file:=/path/to/my.yaml
source "$(dirname "$0")/env.sh"
case "$1" in
    warehouse|warehouse_boxes)
        route="$1"; shift
        set -- "params_file:=$(ros2 pkg prefix people_follower)/share/people_follower/config/human_waypoints_${route}.yaml" "$@" ;;
esac
# Gazebo's VelocityControl keeps the last command, and the bridge stops together
# with the node, so on exit tell Gazebo directly to stop the person.
trap 'gz topic -t /model/human/cmd_vel -m gz.msgs.Twist -p "linear: {x: 0}" 2>/dev/null' EXIT
ros2 launch people_follower human_waypoints.launch.py "$@"
