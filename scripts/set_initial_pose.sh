#!/usr/bin/env bash
# Tell AMCL where the robot is (same as RViz "2D Pose Estimate").
# Usage: ./scripts/set_initial_pose.sh <x> <y> [direction_deg]   (default 0 0 0 = default spawn)
source "$(dirname "$0")/env.sh"
X=${1:-0.0}; Y=${2:-0.0}; DEG=${3:-0}
read -r QZ QW < <(awk -v d="$DEG" 'BEGIN { r = d * 3.14159265 / 180; printf "%f %f\n", sin(r/2), cos(r/2) }')
# Covariance matches RViz's 2D Pose Estimate (0.25 m^2 in x/y, ~0.07 rad^2 in yaw)
COV="[0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06853891945200942]"
# This can see AMCL before AMCL sees it, and then every message is lost even though publishing
# succeeded. So check AMCL really has a pose (it publishes /amcl_pose once it does) and retry.
# The timeouts stop this from waiting forever when AMCL isn't running.
for try in 1 2 3 4; do
    timeout -k 2 20 ros2 topic pub -w 1 -r 1 --times 3 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {frame_id: map}, pose: {pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}}, covariance: $COV}}" \
        > /dev/null 2>&1
    if timeout -k 2 8 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable \
           /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped 2>/dev/null | grep -q frame_id; then
        echo "AMCL has the initial pose (x=$X y=$Y direction=$DEG)"
        exit 0
    fi
    echo "AMCL hasn't confirmed the pose yet (try $try of 4)"
done
echo "Initial pose not confirmed - is AMCL running (launch_nav.sh)?" >&2
exit 1
