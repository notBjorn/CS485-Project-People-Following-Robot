#!/usr/bin/env bash
# Walk the person in person_world through the waypoints in
# src/people_follower/config/human_waypoints.yaml (needs launch_sim.sh world:=person_world).
# Don't run it together with launch_human_teleop.sh: both drive the person.
# Extra args pass through, e.g.: ./scripts/launch_human_waypoints.sh params_file:=/path/to/my.yaml
source "$(dirname "$0")/env.sh"
# Gazebo's VelocityControl keeps the last command, and the bridge stops together
# with the node, so on exit tell Gazebo directly to stop the person.
trap 'gz topic -t /model/human/cmd_vel -m gz.msgs.Twist -p "linear: {x: 0}" 2>/dev/null' EXIT
ros2 launch people_follower human_waypoints.launch.py "$@"
