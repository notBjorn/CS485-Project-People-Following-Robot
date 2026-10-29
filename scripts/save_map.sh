#!/usr/bin/env bash
# Save the map SLAM has built so far into src/people_follower/maps/<name>.pgm/.yaml
# Usage: ./scripts/save_map.sh [name]   (default: box_room - name it after the world)
# Run ./scripts/build.sh afterwards so localization can find it.
source "$(dirname "$0")/env.sh"
NAME=${1:-box_room}
ros2 run nav2_map_server map_saver_cli -f "$PROJECT_ROOT/src/people_follower/maps/$NAME" \
    --ros-args -p use_sim_time:=true
