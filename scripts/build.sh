#!/usr/bin/env bash
# Build the colcon workspace (run from anywhere).
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y -r || true
colcon build --symlink-install "$@"

# TIAGo's model comes from apt (via rosdep above) or from scripts/fetch_tiago_source.sh
source install/setup.bash
if ! ros2 pkg prefix easynav_playground_tiago_description >/dev/null 2>&1; then
    echo
    echo "WARNING: TIAGo's model (easynav_playground_tiago_description) is not installed."
    echo "  Try:  sudo apt install ros-jazzy-easynav-playground-tiago-worlds"
    echo "  or build it from source:  ./scripts/fetch_tiago_source.sh"
    echo "  (robot:=turtlebot4 still works without it.)"
fi
