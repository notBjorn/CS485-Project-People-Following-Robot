#!/usr/bin/env bash
# Health check for a running simulation: topics, sensor rates, TF, Nav2 state.
# Run in a separate terminal while launch_sim/launch_slam/launch_nav is up.
source "$(dirname "$0")/env.sh"
ok()   { echo "  [OK]   $*"; }
bad()  { echo "  [--]   $*"; }

TOPICS=$(ros2 topic list 2>/dev/null)
NODES=$(ros2 node list 2>/dev/null)
if [ -z "$TOPICS" ]; then
    echo "No ROS topics found - is the simulation running (with the same ROS_DOMAIN_ID=$ROS_DOMAIN_ID)?"
    exit 1
fi

echo "Sensor and drive topics (rates measured over ~6 s; the sim can be slow):"
for t in /clock /scan /oakd/rgb/preview/image_raw /odom; do
    if ! grep -qx "$t" <<<"$TOPICS"; then bad "$t missing"; continue; fi
    rate=$(timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -o 'average rate: [0-9.]*' | tail -1)
    if [ -n "$rate" ]; then ok "$t  ($rate Hz)"; else bad "$t exists but no messages in 6 s"; fi
done
grep -qx /cmd_vel <<<"$TOPICS" && ok "/cmd_vel" || bad "/cmd_vel missing"

echo "Map and position (TF):"
grep -qx /map <<<"$TOPICS" && ok "/map published" || bad "/map missing (only with slam or localization)"
if timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -q "Translation"; then
    ok "map -> base_link transform (robot is localized)"
else
    bad "no map -> base_link transform (no SLAM/localization, or initial pose not set)"
fi

echo "Navigation nodes:"
found=0
for n in slam_toolbox amcl map_server planner_server controller_server bt_navigator; do
    grep -qx "/$n" <<<"$NODES" || continue
    found=1
    state=$(timeout 5 ros2 lifecycle get "/$n" 2>/dev/null | awk '{print $1}')
    if [ "$n" = slam_toolbox ] || [ "$state" = active ]; then ok "/$n ${state:+($state)}"
    else bad "/$n is ${state:-not responding} (expected active)"; fi
done
[ $found = 0 ] && echo "  (none running - start with launch_slam.sh or launch_nav.sh)"
exit 0
