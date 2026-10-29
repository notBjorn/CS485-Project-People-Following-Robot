#!/usr/bin/env bash
# Send one Nav2 goal from the terminal (no Python node, no RViz clicking).
# Usage: ./scripts/send_goal.sh <x> <y> [direction_deg]
#   direction: 0 = facing +x (TurtleBot4Directions.NORTH), 90 = +y, ...
# Needs launch_nav.sh running and the robot localized.
source "$(dirname "$0")/env.sh"
if [ $# -lt 2 ]; then echo "Usage: $0 <x> <y> [direction_deg]"; exit 1; fi
X=$1; Y=$2; DEG=${3:-0}
read -r QZ QW < <(awk -v d="$DEG" 'BEGIN { r = d * 3.14159265 / 180; printf "%f %f\n", sin(r/2), cos(r/2) }')
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}}}}"
