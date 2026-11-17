#!/usr/bin/env bash
# Follow the person with UWB (uwb_localizer + uwb_follower). Start the sim first
# (for the TurtleBot 4, facing away from the dock so it doesn't drive over it):
#   ./scripts/launch_sim.sh world:=simple_colored_warehouse yaw:=3.14159
#   ./scripts/launch_follow.sh                         # (TurtleBot 4: undocks,) then follows
#   ./scripts/launch_human_waypoints.sh warehouse      # third terminal: the person walks
# Works with either robot: it checks which one is running.
# Keep the person still for the first ~3 s: the localizer calibrates the anchors then.
# Stops 0.7 m (TIAGo) or 0.5 m (TurtleBot 4) behind the person, slowing to a creep
# over the last 0.5 m.
# Extra args pass through, e.g.: ./scripts/launch_follow.sh follow_distance:=1.0
source "$(dirname "$0")/env.sh"
ROBOT=$(detect_robot)
if [ -z "$ROBOT" ]; then
    echo "No robot found - start the simulation first (launch_sim.sh)" >&2
    exit 1
fi
echo "Robot: $ROBOT"
if [ "$ROBOT" = turtlebot4 ]; then
    echo "Undocking..."
    timeout 60 ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" >/dev/null 2>&1 \
        || echo "Undock didn't confirm (already undocked?); continuing"
fi
# Stop the robot on exit even if the follower didn't get to send its last command.
trap 'ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}" >/dev/null 2>&1' EXIT
ros2 launch people_follower uwb_follow.launch.py robot:="$ROBOT" "$@"
