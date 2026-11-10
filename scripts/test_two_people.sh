#!/usr/bin/env bash
# Short camera-following test with a second person, in simple_colored_warehouse.
#
#   ./scripts/test_two_people.sh               # with real-world camera errors (noise:=true)
#   ./scripts/test_two_people.sh noise:=false  # clean camera
#
# The robot (spawned facing away from the dock) undocks, finds the person and follows
# them round the blue box in room 1 (NE), through the north door and round the red box
# in room 2 (NW). A second person, human2, stands in room 2 at (-7, 4); when the target
# turns down x = -3.5 there, human2 starts walking back and forth along y = 4, crossing
# just behind the target, between them and the robot. The test ends when the target
# reaches room 3 (SW). Takes about 6 minutes. Everything it starts is stopped at the end.
#
# Results (scripts/follow_test.py, against Gazebo's true poses): following distance,
# who the follower was really following and when it changed, closest approach to each
# person, and bumps. Logs: log/test_two_people/.
source "$(dirname "$0")/env.sh"
source "$(dirname "$0")/sim_start.sh"
if [ ! -x "$YOLO_VENV/bin/python3" ]; then
    echo "No YOLO environment at $YOLO_VENV - run ./scripts/setup_yolo.sh first" >&2
    exit 1
fi
LOGS="$PROJECT_ROOT/log/test_two_people"
mkdir -p "$LOGS"
CONFIG="$(ros2 pkg prefix people_follower)/share/people_follower/config"
NOISE="${1:-noise:=true}"
PIDS=()

cleanup() {
    echo "Stopping everything..."
    for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null; done
    sleep 3
    stop_sim >/dev/null 2>&1
    "$(dirname "$0")/kill_sim.sh" >/dev/null 2>&1
}
trap cleanup EXIT

echo "Starting the simulation (log/test_two_people/sim.log)..."
start_sim "$LOGS/sim.log" world:=simple_colored_warehouse rviz:=false yaw:=3.14159 || exit 1

echo "Adding the second person (human2) in room 2..."
"$(dirname "$0")/spawn_person.sh" human2 -7 4 0 > "$LOGS/spawn.log" 2>&1 \
    || { echo "Spawning human2 failed, see $LOGS/spawn.log"; exit 1; }

echo "Undocking and starting the camera follower ($NOISE)..."
"$(dirname "$0")/launch_camera_follow.sh" "$NOISE" > "$LOGS/follow.log" 2>&1 &
PIDS+=($!)
for _ in $(seq 1 120); do grep -q 'Following person' "$LOGS/follow.log" && break; sleep 1; done
if ! grep -q 'Following person' "$LOGS/follow.log"; then
    echo "The follower never found the person; see $LOGS/follow.log"
    exit 1
fi
echo "Person found; they start walking."

# The target walks now; human2 starts walking when follow_test.py sees the target reach
# (-3.5, 6.5). kill_sim.sh at the end stops both walkers along with everything else.
"$(dirname "$0")/launch_human_waypoints.sh" two_rooms > "$LOGS/target_walk.log" 2>&1 &
PIDS+=($!)

python3 "$(dirname "$0")/follow_test.py" \
    --start-cmd "'$(dirname "$0")/launch_human_waypoints.sh' person:=human2 params_file:='$CONFIG/human2_crossing.yaml' > '$LOGS/human2_walk.log' 2>&1" \
    | tee "$LOGS/results.txt"
echo "Follower and tracker events:"
sed 's/\x1b\[[0-9;]*m//g' "$LOGS/follow.log" | grep -E '\[(person_tracker|person_follower)\]' \
    | sed 's/^.*\]: //' | tee "$LOGS/events.txt" | sed -E 's/\([-0-9., ]+\)/(x, y)/; s/person [0-9]+/person N/' \
    | sort | uniq -c | sort -rn
