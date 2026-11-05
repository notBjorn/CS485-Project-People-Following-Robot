#!/usr/bin/env bash
# Record a test run (sensors, position, commands) to bags/<name>_<time> for replay.
# Usage: ./scripts/record_bag.sh [name] [--no-camera]    Ctrl+C to stop recording.
# Replay with: ros2 bag play bags/<folder> --clock
source "$(dirname "$0")/env.sh"
NAME=run; CAMERA=1
for arg in "$@"; do
    case $arg in
        --no-camera) CAMERA=0 ;;
        *) NAME=$arg ;;
    esac
done
TOPICS=(/clock /scan /odom /tf /tf_static /cmd_vel /map /amcl_pose /plan /goal_pose)
# Camera images make bags large quickly
[ $CAMERA = 1 ] && TOPICS+=(/oakd/rgb/preview/image_raw /oakd/rgb/preview/camera_info)
OUT="$PROJECT_ROOT/bags/${NAME}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$PROJECT_ROOT/bags"
echo "Recording to $OUT - Ctrl+C to stop"
ros2 bag record -o "$OUT" "${TOPICS[@]}"
