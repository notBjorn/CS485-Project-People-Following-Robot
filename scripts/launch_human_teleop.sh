#!/usr/bin/env bash
# Drive the person in person_world from the keyboard (run in its own terminal,
# with launch_sim.sh world:=person_world running). Same keys as launch_teleop.sh.
# Bridges ROS /model/human/cmd_vel to the person's VelocityControl in Gazebo.
source "$(dirname "$0")/env.sh"
# Run the bridge binary directly (not via `ros2 run`) so killing $BRIDGE_PID stops it
"$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge" \
    '/model/human/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' > /dev/null 2>&1 &
BRIDGE_PID=$!
trap 'kill $BRIDGE_PID 2>/dev/null' EXIT
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/model/human/cmd_vel
