# Shared environment for the CS485 project scripts (sourced by each one).
# Scripts don't read ~/.bashrc, so set everything here.
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
else
    echo "Workspace not built yet - run scripts/build.sh first" >&2
fi
# Discovery uses the ROS default (subnet). Don't set ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST:
# Nav2's nodes can't find each other's services with it in this large a launch.
# On a shared network, give each person their own ID: export ROS_DOMAIN_ID=<0-101>
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
export GZ_IP=127.0.0.1
