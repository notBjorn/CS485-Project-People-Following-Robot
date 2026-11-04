#!/usr/bin/env bash
# Stop every simulation process: Gazebo, bridges, RViz, Nav2/SLAM, our nodes.
# Use when Ctrl+C left something running and the next launch misbehaves.
# Note: also stops any other ROS 2 nodes running on this machine (e.g. teleop).
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PATTERNS=("ros2 launch" "ros2 run" "gz sim" "/opt/ros/jazzy/lib/" "/opt/ros/jazzy/bin/ros2" "$PROJECT_ROOT/install/")

# Ask launch files to shut down cleanly first
pkill -INT -f "ros2 launch" && sleep 3
for p in "${PATTERNS[@]}"; do pkill -TERM -f "$p"; done
sleep 2
for p in "${PATTERNS[@]}"; do pkill -KILL -f "$p"; done

# Force-killing can leave a locked Fast DDS port behind, which would hang the next launch
"$PROJECT_ROOT/scripts/clean_dds.sh"

left=$(for p in "${PATTERNS[@]}"; do pgrep -af "$p"; done | sort -u)
if [ -n "$left" ]; then
    echo "Still running:"; echo "$left"; exit 1
fi
echo "Simulation processes stopped."
