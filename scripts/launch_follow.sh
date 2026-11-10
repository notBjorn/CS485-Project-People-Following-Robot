#!/usr/bin/env bash
# Follow the person with UWB (uwb_localizer + uwb_follower). Start the sim first,
# facing away from the dock so the robot doesn't drive over it:
#   ./scripts/launch_sim.sh world:=simple_colored_warehouse yaw:=3.14159
#   ./scripts/launch_follow.sh                         # undocks, then follows
#   ./scripts/launch_human_waypoints.sh warehouse      # third terminal: the person walks
# Keep the person still for the first ~3 s: the localizer calibrates the anchors then.
# Stops 0.5 m behind the person, slowing to a creep over the last 0.5 m.
# Extra args pass through, e.g.: ./scripts/launch_follow.sh follow_distance:=1.0
source "$(dirname "$0")/env.sh"
echo "Undocking..."
timeout 60 ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" >/dev/null 2>&1 \
    || echo "Undock didn't confirm (already undocked?); continuing"
# Stop the robot on exit even if the follower didn't get to send its last command.
trap 'ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}" >/dev/null 2>&1' EXIT
ros2 launch people_follower uwb_follow.launch.py "$@"
