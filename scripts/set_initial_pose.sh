#!/usr/bin/env bash
# Tell AMCL where the robot is (same as RViz "2D Pose Estimate").
# Usage: ./scripts/set_initial_pose.sh <x> <y> [direction_deg]   (default 0 0 0 = default spawn)
source "$(dirname "$0")/env.sh"
X=${1:-0.0}; Y=${2:-0.0}; DEG=${3:-0}
read -r QZ QW < <(awk -v d="$DEG" 'BEGIN { r = d * 3.14159265 / 180; printf "%f %f\n", sin(r/2), cos(r/2) }')
# Covariance matches RViz's 2D Pose Estimate (0.25 m^2 in x/y, ~0.07 rad^2 in yaw)
COV="[0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06853891945200942]"
ros2 topic pub --once -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}}, covariance: $COV}}"
