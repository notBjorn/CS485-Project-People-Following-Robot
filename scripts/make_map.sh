#!/usr/bin/env bash
# Guided mapping: starts the sim with SLAM, opens teleop in a new terminal,
# saves the map when you press Enter, then rebuilds so localization can use it.
# Usage: ./scripts/make_map.sh [world] [sim args]   (default: box_room; the map is named after the world)
#   e.g. ./scripts/make_map.sh person_world robot:=turtlebot4   (TIAGo is the default)
source "$(dirname "$0")/env.sh"
source "$(dirname "$0")/sim_start.sh"
WORLD=${1:-box_room}
shift || true

LOG="$PROJECT_ROOT/log/make_map_launch.log"
stop_sim_on_exit
echo "Simulation + SLAM starting for '$WORLD' (output in $LOG)."
start_sim "$LOG" slam:=true world:="$WORLD" "$@" || exit 1

TELEOP="$PROJECT_ROOT/scripts/launch_teleop.sh"
if command -v gnome-terminal >/dev/null; then
    gnome-terminal --title="Teleop - keep this window focused to drive" -- "$TELEOP"
elif command -v x-terminal-emulator >/dev/null; then
    x-terminal-emulator -e "$TELEOP" &
else
    echo "Open another terminal and run: $TELEOP"
fi

if [ "$(detect_robot)" = turtlebot4 ]; then
    echo "The robot starts docked and facing the dock, so undock it first (in another terminal):"
    echo "  source scripts/env.sh && ros2 action send_goal /undock irobot_create_msgs/action/Undock \"{}\""
fi
echo "Then drive around the whole area in the teleop window, watching the map grow in RViz."
echo "Turn slowly (press c a few times to slow turns): fast turns rotate the map."
read -rp "Press Enter when the map looks complete to save it (Ctrl+C to quit without saving)... "

"$PROJECT_ROOT/scripts/save_map.sh" "$WORLD" || { echo "Map save failed"; exit 1; }
stop_sim
"$PROJECT_ROOT/scripts/build.sh"
echo "Saved src/people_follower/maps/$WORLD.yaml/.pgm - commit them so the team has the map."
