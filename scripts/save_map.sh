#!/usr/bin/env bash
# Save the map SLAM has built so far into src/people_follower/maps/<name>.pgm/.yaml
# Usage: ./scripts/save_map.sh [name]   (default: box_room - name it after the world)
# Run ./scripts/build.sh afterwards so localization can find it.
source "$(dirname "$0")/env.sh"
NAME=${1:-box_room}
# map_saver only waits a couple of seconds for the map, so the first try can fail.
for try in 1 2 3; do
    ros2 run nav2_map_server map_saver_cli -f "$PROJECT_ROOT/src/people_follower/maps/$NAME" \
        --ros-args -p use_sim_time:=true && exit 0
    [ "$try" -lt 3 ] && { echo "Map not saved (try $try of 3) - trying again in 5 s"; sleep 5; }
done
echo "Couldn't save the map. Is SLAM running (launch_slam.sh)?" >&2
exit 1
