# Shared steps for demo_follow.sh and demo_two_people.sh (sourced, not run).
# Everything a demo starts is stopped when it exits (Ctrl+C, or a failed step).

# Build first, so the demo always runs this branch's code; it takes a few seconds when
# nothing has changed. A package in install/ that this branch doesn't have was built on
# another branch, so then build/ and install/ are cleared and the build starts fresh.
_build_log="$(mktemp)"
echo "Building the workspace..."
if ! (
    source /opt/ros/jazzy/setup.bash
    cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
    packages=$(colcon list --names-only --base-paths src)
    for dir in install/*/; do
        [ -d "$dir" ] || continue
        if ! grep -qx "$(basename "$dir")" <<< "$packages"; then
            echo "install/ has $(basename "$dir") from another branch; building from scratch"
            rm -rf build install
            break
        fi
    done
    ./scripts/build.sh
) > "$_build_log" 2>&1; then
    tail -n 30 "$_build_log" >&2
    echo "The build failed (full output: $_build_log)." >&2
    exit 1
fi
rm -f "$_build_log"

source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
source "$(dirname "${BASH_SOURCE[0]}")/sim_start.sh"
SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="$(ros2 pkg prefix people_follower)/share/people_follower/config"
DEMO_PIDS=()

# Set up YOLO the first time (or if a previous setup didn't finish): the Python
# environment and the model. Several minutes and about 3 GB; later runs skip it.
if ! "$YOLO_VENV/bin/python3" -c 'import ultralytics' > /dev/null 2>&1 \
        || [ ! -f "$PROJECT_ROOT/models/yolo26n.pt" ]; then
    echo "Setting up YOLO for person detection (first run only, about 3 GB)..."
    if ! "$SCRIPTS/setup_yolo.sh"; then
        echo "YOLO setup failed - see the messages above" >&2
        exit 1
    fi
fi

# demo_setup <name>: log folder log/<name>/, and clean-up on exit.
demo_setup() {
    LOGS="$PROJECT_ROOT/log/$1"
    mkdir -p "$LOGS"
    trap demo_cleanup EXIT
    trap 'exit 0' INT TERM
}

demo_cleanup() {
    echo
    echo "Stopping the demo..."
    for pid in "${DEMO_PIDS[@]}"; do kill -INT "$pid" 2>/dev/null; done
    sleep 2
    stop_sim >/dev/null 2>&1
    "$SCRIPTS/kill_sim.sh" >/dev/null 2>&1
    pkill -f "rqt_image_view /person/image_annotated" 2>/dev/null
    echo "Stopped. Logs: $LOGS"
}

# demo_start_sim: the warehouse, robot at the origin facing away from the dock (it turns
# round to look for the person, so it never drives over the dock).
demo_start_sim() {
    echo "Starting Gazebo with the warehouse (log: $LOGS/sim.log)..."
    start_sim "$LOGS/sim.log" world:=simple_colored_warehouse rviz:=false yaw:=3.14159 || exit 1
}

# demo_start_follower <noise:=true|false>: undock, then YOLO + depth + tracker + follower
# + safety guard. Waits until the robot has picked the person to follow.
demo_start_follower() {
    echo "Undocking and starting the camera follower ($1)..."
    "$SCRIPTS/launch_camera_follow.sh" "$1" > "$LOGS/follow.log" 2>&1 &
    DEMO_PIDS+=($!)
    for _ in $(seq 1 120); do grep -q 'Following person' "$LOGS/follow.log" && break; sleep 1; done
    if ! grep -q 'Following person' "$LOGS/follow.log"; then
        echo "The robot never found the person; see $LOGS/follow.log" >&2
        exit 1
    fi
    echo "The robot has found the person."
}

# demo_camera_view: a window with the robot's camera and YOLO's boxes.
demo_camera_view() {
    ros2 run rqt_image_view rqt_image_view /person/image_annotated > /dev/null 2>&1 &
    DEMO_PIDS+=($!)
}

# demo_watch <walk log>...: print what the robot is doing until Ctrl+C.
demo_watch() {
    echo
    echo "Demo running. Watch Gazebo and the camera window; events below. Ctrl+C to stop."
    echo
    tail -n 0 -F "$LOGS/follow.log" "$@" 2>/dev/null | sed -u 's/\x1b\[[0-9;]*m//g' \
        | grep --line-buffered -E '\[(person_tracker|person_follower|safety_guard|human_waypoints|human2_waypoints)\]|^\[[0-9]+ s\]' \
        | sed -u -E 's/^\[[a-z0-9_]+-[0-9]+\] \[[A-Z]+\] \[[0-9.]+\] \[([a-z0-9_]+)\]: /  \1: /'
}
