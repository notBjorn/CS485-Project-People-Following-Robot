#!/usr/bin/env bash
# One-time setup for person detection with YOLO (people_follower/person_detector).
#
# Creates a Python virtual environment at .venv-yolo with Ultralytics (PyTorch with
# CUDA, about 3 GB), and downloads the YOLO model into models/. Both are git-ignored.
# Ubuntu 24.04 doesn't allow pip installs into the system Python, hence the venv; it
# is made with --system-site-packages so it still sees ROS 2 (rclpy, cv_bridge).
#
# Needs python3.12-venv:  sudo apt install python3.12-venv
# Run again any time to update; it is safe to repeat.
set -e
source "$(dirname "$0")/env.sh"
MODEL="${1:-yolo26n.pt}"

if ! python3 -c 'import ensurepip' 2>/dev/null; then
    echo "Python can't create virtual environments yet. Install it with:"
    echo "  sudo apt install python3.12-venv"
    exit 1
fi

if [ ! -x "$YOLO_VENV/bin/python" ]; then
    echo "Creating $YOLO_VENV ..."
    python3 -m venv --system-site-packages "$YOLO_VENV"
fi

echo "Installing Ultralytics (first time: several minutes, ~3 GB) ..."
"$YOLO_VENV/bin/python" -m pip install --upgrade pip
# numpy<2: ROS Jazzy's cv_bridge is built against Ubuntu's NumPy 1.x and fails to
# import next to NumPy 2. Headless OpenCV, so pip's OpenCV doesn't bring its own Qt.
"$YOLO_VENV/bin/python" -m pip install --upgrade ultralytics "numpy<2" opencv-python-headless

echo "Downloading $MODEL into $PROJECT_ROOT/models ..."
mkdir -p "$PROJECT_ROOT/models"
(cd "$PROJECT_ROOT/models" && "$YOLO_VENV/bin/python" -c "from ultralytics import YOLO; YOLO('$MODEL')")

echo
echo "Checking the install:"
"$YOLO_VENV/bin/python" - <<PY
import numpy, torch, ultralytics, cv2
from cv_bridge import CvBridge
import rclpy
print(f"  ultralytics {ultralytics.__version__}, torch {torch.__version__}, numpy {numpy.__version__}, cv2 {cv2.__version__}")
print(f"  GPU: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'not available, will use the CPU'}")
print("  ROS 2 rclpy and cv_bridge import fine")
PY
ls -l "$PROJECT_ROOT/models/$MODEL"
echo "Done. Run ./scripts/build.sh, then ./scripts/launch_person_detector.sh"
