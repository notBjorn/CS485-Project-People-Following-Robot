#!/usr/bin/env bash
# Demo 2 (UWB): UWB following with a second person, in simple_colored_warehouse.
#
# The robot follows the person wearing the UWB tag round the blue box in room 1 (NE),
# through the north door and round the red box in room 2 (NW), then into room 3 (SW).
# A second person, human2, without a tag, stands in room 2 at (-7, 4); when the target
# turns down x = -3.5 there, human2 starts walking back and forth across their path,
# between them and the robot. About 5 minutes until the target reaches room 3; the demo
# keeps running until Ctrl+C, then prints who the robot was following
# (scripts/follow_test.py).
#
#   ./scripts/demo_uwb_two_people.sh                  # TIAGo
#   ./scripts/demo_uwb_two_people.sh robot:=turtlebot4
#
# Opens Gazebo. Ctrl+C stops everything. Logs: log/demo_uwb_two_people/.
# Builds the workspace first (./scripts/build.sh), so there is no separate build step.
source "$(dirname "$0")/demo_uwb_common.sh"
demo_setup demo_uwb_two_people
demo_start_sim

echo "Adding the second person (human2, no UWB tag) in room 2..."
"$SCRIPTS/spawn_person.sh" human2 -7 4 0 > "$LOGS/spawn.log" 2>&1 \
    || { echo "Spawning human2 failed, see $LOGS/spawn.log" >&2; exit 1; }

demo_start_follower

echo "The person starts walking: room 1, room 2, then into room 3."
"$SCRIPTS/launch_human_waypoints.sh" two_rooms > "$LOGS/walk.log" 2>&1 &
DEMO_PIDS+=($!)

# follow_test.py starts human2 when the target reaches (-3.5, 6.5), and on Ctrl+C
# prints who the robot was following (it never ends the demo by itself here).
# --robot: the robot's Gazebo model name, tiago or turtlebot4. (Bumps are only counted on
# the TurtleBot 4, whose bumper publishes /hazard_detection.)
python3 "$SCRIPTS/follow_test.py" --robot "$ROBOT" --target-topic /uwb/person --timeout 100000 --end-x-below=-1e9 \
    --start-cmd "'$SCRIPTS/launch_human_waypoints.sh' person:=human2 params_file:='$CONFIG/human2_crossing.yaml' > '$LOGS/human2_walk.log' 2>&1" \
    > "$LOGS/who_followed.txt" 2>&1 &
WATCHER=$!
DEMO_PIDS+=($WATCHER)

# On Ctrl+C: let follow_test.py print its summary before everything stops.
summary() {
    kill -TERM "$WATCHER" 2>/dev/null
    for _ in $(seq 1 10); do kill -0 "$WATCHER" 2>/dev/null || break; sleep 0.5; done
    echo
    sed -n '/=== Results ===/,$p' "$LOGS/who_followed.txt"
    exit 0
}
trap summary INT TERM
demo_watch "$LOGS/walk.log" "$LOGS/human2_walk.log" "$LOGS/who_followed.txt"
