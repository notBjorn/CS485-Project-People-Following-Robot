# Shared simulation startup, sourced by the launch scripts after env.sh.
#
# Startup can stall: no robot appears, and RViz sits open with nothing to show. The cause
# we've traced is a locked Fast DDS port left by a killed node (see clean_dds.sh), so
# start_sim clears those before launching. It then waits for the robot's drive controller,
# relaunching if it doesn't appear, and prints a message each time it does.
#
# Three limits, in seconds, all overridable:
#   SIM_START_TIMEOUT  launch -> server's first line
#   SIM_WORLD_TIMEOUT  server's first line -> world loaded. Long, because the first run
#                      of a world can download models.
#   SIM_ROBOT_TIMEOUT  world loaded -> robot's drive controller running (the TurtleBot 4's
#                      diffdrive_controller or TIAGo's mobile_base_controller). Longer
#                      than the TurtleBot 4 needs, because TIAGo starts more controllers.
SIM_START_TIMEOUT=${SIM_START_TIMEOUT:-30}
SIM_WORLD_TIMEOUT=${SIM_WORLD_TIMEOUT:-300}
SIM_ROBOT_TIMEOUT=${SIM_ROBOT_TIMEOUT:-90}
SIM_START_ATTEMPTS=3

# True if any simulation process is still running.
sim_running() {
    # The [r] stops grep matching its own command line in the process list.
    ps -eo cmd | grep -qE '^gz sim|^ruby .*gz sim|/rviz2/[r]viz2 '
}

# wait_for_log <log> <text> <seconds>: 0 = found, 1 = timed out, 2 = launch exited.
wait_for_log() {
    local t
    for ((t = 0; t < $3; t++)); do
        grep -aq "$2" "$1" && return 0
        kill -0 "$SIM_PID" 2>/dev/null || return 2
        sleep 1
    done
    grep -aq "$2" "$1"
}

# start_sim <log file> [launch args...]: returns once the robot is ready; sets SIM_PID.
start_sim() {
    local log=$1 attempt rc
    shift
    mkdir -p "$(dirname "$log")"
    for ((attempt = 1; attempt <= SIM_START_ATTEMPTS; attempt++)); do
        "$PROJECT_ROOT/scripts/clean_dds.sh"
        : > "$log"
        ros2 launch people_follower sim.launch.py "$@" >> "$log" 2>&1 &
        SIM_PID=$!
        wait_for_log "$log" 'Gazebo Sim Server v' "$SIM_START_TIMEOUT" &&
            wait_for_log "$log" 'Serving world controls' "$SIM_WORLD_TIMEOUT" &&
            wait_for_log "$log" 'Configured and activated \(diffdrive\|mobile_base_controller\)' \
                "$SIM_ROBOT_TIMEOUT"
        rc=$?
        [ "$rc" -eq 0 ] && return 0
        if [ "$rc" -eq 2 ]; then
            echo "Launch exited early - last lines of $log:" >&2
            tail -20 "$log" >&2
            SIM_PID=
            sim_running && "$PROJECT_ROOT/scripts/kill_sim.sh" > /dev/null 2>&1
            return 1
        fi
        echo "Simulation startup stalled (attempt $attempt of $SIM_START_ATTEMPTS)." \
             "Stopping it and any other ROS nodes on this machine." \
             "Its log is kept as $log.stalled" >&2
        cp "$log" "$log.stalled"
        "$PROJECT_ROOT/scripts/kill_sim.sh" > /dev/null 2>&1
        SIM_PID=
    done
    echo "The simulation didn't start after $SIM_START_ATTEMPTS attempts. See $log" >&2
    return 1
}

# Stop what start_sim started. Background jobs ignore Ctrl+C's SIGINT, so use SIGTERM.
# The launch can exit and leave Gazebo or RViz behind, so check for those too.
stop_sim() {
    trap - INT TERM EXIT
    # Helpers left running in the background, such as launch_nav.sh's pose setter
    local helpers
    helpers=$(jobs -p)
    [ -n "$helpers" ] && kill $helpers 2>/dev/null
    [ -n "$SIM_PID" ] || return 0
    echo "Stopping simulation..."
    kill -TERM "$SIM_PID" 2>/dev/null
    for _ in $(seq 1 15); do kill -0 "$SIM_PID" 2>/dev/null || break; sleep 1; done
    if kill -0 "$SIM_PID" 2>/dev/null || sim_running; then
        "$PROJECT_ROOT/scripts/kill_sim.sh"
    fi
    "$PROJECT_ROOT/scripts/clean_dds.sh"
    SIM_PID=
    return 0
}

# Stop the sim when the calling script exits or gets Ctrl+C.
stop_sim_on_exit() {
    SIM_PID=
    trap stop_sim EXIT
    trap "stop_sim; exit 130" INT TERM
}

# Show the launch output in this terminal until the launch ends.
follow_sim() {
    tail -n +1 -f --pid="$SIM_PID" "$1" &
    wait "$SIM_PID"
}
