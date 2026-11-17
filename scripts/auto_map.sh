#!/usr/bin/env bash
# The robot maps a world by itself: SLAM + Nav2, and auto_explorer drives it to the
# edges of the known map until none are left, then the map is saved and the workspace
# rebuilt (so launch_nav.sh / localization can use it). Watch the map grow in RViz.
#   ./scripts/auto_map.sh [world] [map name] [sim args]   (defaults: box_room, named after the world)
#   e.g. ./scripts/auto_map.sh simple_colored_warehouse
#        ./scripts/auto_map.sh box_room box_room robot:=turtlebot4   (TIAGo is the default)
# The map is saved when exploration finishes, including when some edges couldn't be
# reached or after the 15-minute limit (it says so: the map is then partial).
# Ctrl+C stops without saving. To save by hand instead, run ./scripts/save_map.sh <name>
# in another terminal while SLAM is running.
source "$(dirname "$0")/env.sh"
source "$(dirname "$0")/sim_start.sh"
WORLD=${1:-box_room}
NAME=${2:-$WORLD}
shift 2 2>/dev/null || shift $#
LOGS="$PROJECT_ROOT/log/auto_map"
mkdir -p "$LOGS"
stop_sim_on_exit

echo "Simulation + SLAM + Nav2 starting for '$WORLD' (output in $LOGS/sim.log)..."
start_sim "$LOGS/sim.log" world:="$WORLD" slam:=true nav2:=true "$@" || exit 1

# TIAGo has no dock. The TurtleBot 4 starts docked: undocking spins the robot round quickly. A fast turn while SLAM is mapping can leave a
# rotated copy of the room in the map, so SLAM ignores new scans during the undock; when
# it resumes, it matches the next scans to the map it already has.
if [ "$(detect_robot)" = turtlebot4 ]; then
    echo "Undocking (SLAM paused)..."
    timeout 20 ros2 service call /slam_toolbox/pause_new_measurements slam_toolbox/srv/Pause \
        >/dev/null 2>&1 || echo "Couldn't pause SLAM; continuing"
    timeout 60 ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" >/dev/null 2>&1 \
        || echo "Undock didn't confirm (already undocked?); continuing"
    sleep 2
    timeout 20 ros2 service call /slam_toolbox/pause_new_measurements slam_toolbox/srv/Pause \
        >/dev/null 2>&1 || echo "Couldn't resume SLAM"
fi

echo "Waiting for Nav2 to be ready..."
for _ in $(seq 1 120); do
    timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q '^active' && break
    sleep 1
done
echo "Exploring (up to 15 minutes)..."
ros2 run people_follower auto_explorer --ros-args -p use_sim_time:=true
case $? in
    0) echo "Map complete." ;;
    2) echo "Map partial: some areas couldn't be reached (or the time limit hit). Saving it anyway." ;;
    *) echo "Exploration stopped before it finished; map not saved."; exit 1 ;;
esac
echo "Saving the map as '$NAME'..."
"$PROJECT_ROOT/scripts/save_map.sh" "$NAME" || { echo "Map save failed"; exit 1; }
stop_sim
"$PROJECT_ROOT/scripts/build.sh"
echo "Saved src/people_follower/maps/$NAME.yaml/.pgm - commit them so the team has the map."
