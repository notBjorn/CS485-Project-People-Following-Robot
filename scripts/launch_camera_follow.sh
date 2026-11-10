#!/usr/bin/env bash
# Follow the person with the camera (YOLO + depth + tracker + person_follower).
# Needs ./scripts/setup_yolo.sh once. Start the sim first, facing away from the dock
# so the robot doesn't drive over it (it turns round to look for the person):
#   ./scripts/launch_sim.sh world:=simple_colored_warehouse yaw:=3.14159
#   ./scripts/launch_camera_follow.sh                       # undocks, then follows
#   ./scripts/launch_human_waypoints.sh warehouse_boxes     # third terminal: the person walks
# Stops 1.0 m behind the person, slowing over the last 1.0 m.
# Extra args pass through, e.g.: ./scripts/launch_camera_follow.sh noise:=true
source "$(dirname "$0")/env.sh"
if [ ! -x "$YOLO_VENV/bin/python3" ]; then
    echo "No YOLO environment at $YOLO_VENV - run ./scripts/setup_yolo.sh first" >&2
    exit 1
fi
echo "Undocking..."
timeout 60 ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" >/dev/null 2>&1 \
    || echo "Undock didn't confirm (already undocked?); continuing"
# Stop the robot on exit even if the follower didn't get to send its last command.
trap 'ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}" >/dev/null 2>&1' EXIT
ros2 launch people_follower camera_follow.launch.py "$@"
