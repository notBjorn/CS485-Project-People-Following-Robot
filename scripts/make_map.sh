#!/usr/bin/env bash
# Guided mapping: starts the sim with SLAM, opens teleop in a new terminal,
# saves the map when you press Enter, then rebuilds so localization can use it.
# Usage: ./scripts/make_map.sh [world]   (default: box_room; the map is named after the world)
source "$(dirname "$0")/env.sh"
WORLD=${1:-box_room}

LOG="$PROJECT_ROOT/log/make_map_launch.log"
mkdir -p "$PROJECT_ROOT/log"
ros2 launch people_follower sim.launch.py slam:=true world:="$WORLD" > "$LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
    trap - INT TERM EXIT
    echo "Stopping simulation..."
    kill -TERM "$LAUNCH_PID" 2>/dev/null   # background jobs ignore SIGINT
    for _ in $(seq 1 15); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
    kill -0 "$LAUNCH_PID" 2>/dev/null && "$PROJECT_ROOT/scripts/kill_sim.sh"
}
trap cleanup EXIT
trap "cleanup; exit 130" INT TERM

TELEOP="$PROJECT_ROOT/scripts/launch_teleop.sh"
if command -v gnome-terminal >/dev/null; then
    gnome-terminal --title="Teleop - keep this window focused to drive" -- "$TELEOP"
elif command -v x-terminal-emulator >/dev/null; then
    x-terminal-emulator -e "$TELEOP" &
else
    echo "Open another terminal and run: $TELEOP"
fi

echo "Simulation + SLAM starting for '$WORLD' (output in $LOG)."
echo "Drive the robot around the whole area in the teleop window, watching the map grow in RViz."
read -rp "Press Enter when the map looks complete to save it (Ctrl+C to quit without saving)... "

"$PROJECT_ROOT/scripts/save_map.sh" "$WORLD" || { echo "Map save failed"; exit 1; }
cleanup
"$PROJECT_ROOT/scripts/build.sh"
echo "Saved src/people_follower/maps/$WORLD.yaml/.pgm - commit them so the team has the map."
