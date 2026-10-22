#!/usr/bin/env bash
# Build the colcon workspace (run from anywhere).
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y -r || true
colcon build --symlink-install "$@"
