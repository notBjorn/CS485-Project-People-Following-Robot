#!/usr/bin/env bash
# Test the safety zone (safety_guard) on its own: drive the robot straight at a wall
# end, the dock (too low for the LiDAR) and a person at full speed, and measure how
# close it got (scripts/safety_test.py, against Gazebo's true poses). About 3 minutes.
#   ./scripts/test_safety.sh               # clean depth camera
#   ./scripts/test_safety.sh noise:=true   # noisy depth camera
# Logs: log/test_safety/. Everything it starts is stopped at the end.
source "$(dirname "$0")/env.sh"
source "$(dirname "$0")/sim_start.sh"
LOGS="$PROJECT_ROOT/log/test_safety"
mkdir -p "$LOGS"
DEPTH=/oakd/rgb/preview/depth
PIDS=()
cleanup() {
    echo "Stopping everything..."
    for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null; done
    sleep 2
    stop_sim >/dev/null 2>&1
    "$(dirname "$0")/kill_sim.sh" >/dev/null 2>&1
}
trap cleanup EXIT

echo "Starting the simulation (log/test_safety/sim.log)..."
start_sim "$LOGS/sim.log" world:=simple_colored_warehouse rviz:=false yaw:=3.14159 || exit 1
echo "Undocking..."
timeout 60 ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" >/dev/null 2>&1
if [ "$1" = "noise:=true" ]; then
    DEPTH=/oakd_noisy/rgb/preview/depth
    ros2 run people_follower sensor_noise --ros-args -p use_sim_time:=true \
        --params-file "$(ros2 pkg prefix people_follower)/share/people_follower/config/sensor_noise.yaml" \
        > "$LOGS/noise.log" 2>&1 &
    PIDS+=($!)
fi
ros2 run people_follower safety_guard --ros-args -p use_sim_time:=true -p depth_topic:=$DEPTH \
    > "$LOGS/guard.log" 2>&1 &
PIDS+=($!)
# The person's odometry bridge isn't needed: poses come from Gazebo directly.
python3 "$(dirname "$0")/safety_test.py" | tee "$LOGS/results.txt"
echo "Safety guard events:"
sed 's/\x1b\[[0-9;]*m//g' "$LOGS/guard.log" | grep '\[safety_guard\]' | sed 's/^.*\]: //' \
    | sed -E 's/[0-9]+\.[0-9]+/N/g' | sort | uniq -c | sort -rn
