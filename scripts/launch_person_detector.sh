#!/usr/bin/env bash
# Detect people in the robot's camera with YOLO (needs ./scripts/setup_yolo.sh once).
# Start the sim first, e.g. ./scripts/launch_sim.sh world:=person_world
# Boxes: /person/detections (vision_msgs/Detection2DArray); positions relative to the
# robot: /person/detections_3d (vision_msgs/Detection3DArray, base_link);
# picture with the boxes drawn: ros2 run rqt_image_view rqt_image_view /person/image_annotated
# Real-world camera/detection errors (config/sensor_noise.yaml): add noise:=true
# Extra args pass through, e.g.: ./scripts/launch_person_detector.sh confidence:=0.5
source "$(dirname "$0")/env.sh"
if [ ! -x "$YOLO_VENV/bin/python3" ]; then
    echo "No YOLO environment at $YOLO_VENV - run ./scripts/setup_yolo.sh first" >&2
    exit 1
fi
ros2 launch people_follower person_detector.launch.py "$@"
