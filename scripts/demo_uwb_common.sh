# Shared steps for demo_uwb_follow.sh and demo_uwb_two_people.sh (sourced, not run).
# Everything a demo starts is stopped when it exits (Ctrl+C, or a failed step).
# The robot is TIAGo unless the demo is run with robot:=turtlebot4.

ROBOT=tiago
for _arg in "$@"; do
    case "$_arg" in
        robot:=*) ROBOT=${_arg#robot:=} ;;
    esac
done

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
    echo "Stopped. Logs: $LOGS"
}

# demo_start_sim [world x y yaw]: a world with the UWB plugin and a person wearing tag 1
# (the robot carries anchors 1-4). Default: the warehouse, robot at the origin facing
# away from the TurtleBot 4's dock so it never drives over it (TIAGo has no dock; it
# faces the same way so the demos match).
demo_start_sim() {
    local world=${1:-simple_colored_warehouse} x=${2:-0.0} y=${3:-0.0} yaw=${4:-3.14159}
    echo "Starting Gazebo with $world and $ROBOT (log: $LOGS/sim.log)..."
    start_sim "$LOGS/sim.log" robot:="$ROBOT" world:="$world" rviz:=false \
        x:="$x" y:="$y" yaw:="$yaw" || exit 1
}

# demo_start_follower: undock (TurtleBot 4), then uwb_localizer + uwb_follower. Waits until the
# localizer has calibrated the anchors (the person must stand still until then).
demo_start_follower() {
    echo "Undocking and starting the UWB follower..."
    "$SCRIPTS/launch_follow.sh" > "$LOGS/follow.log" 2>&1 &
    DEMO_PIDS+=($!)
    for _ in $(seq 1 120); do grep -q 'Calibrated' "$LOGS/follow.log" && break; sleep 1; done
    if ! grep -q 'Calibrated' "$LOGS/follow.log"; then
        echo "The UWB localizer never calibrated; see $LOGS/follow.log" >&2
        exit 1
    fi
    echo "UWB anchors calibrated; the robot is tracking the person's tag."
}

# demo_watch <log>...: print what the robot is doing until Ctrl+C.
demo_watch() {
    echo
    echo "Demo running. Watch Gazebo; events below. Ctrl+C to stop."
    echo
    tail -n 0 -F "$LOGS/follow.log" "$@" 2>/dev/null | sed -u 's/\x1b\[[0-9;]*m//g' \
        | grep --line-buffered -E '\[(uwb_localizer|uwb_follower|human_waypoints|human2_waypoints)\]|^\[[0-9]+ s\]' \
        | sed -u -E 's/^\[[a-z0-9_]+-[0-9]+\] \[[A-Z]+\] \[[0-9.]+\] \[([a-z0-9_]+)\]: /  \1: /'
}
