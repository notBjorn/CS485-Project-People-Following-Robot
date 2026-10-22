# Shared environment for the CS485 project scripts (sourced by each one).
# Scripts don't read ~/.bashrc, so set everything here.
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
else
    echo "Workspace not built yet - run scripts/build.sh first" >&2
fi
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_IP=127.0.0.1
