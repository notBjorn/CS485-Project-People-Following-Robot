#!/usr/bin/env bash
# Add another person to the running simulation (models/person.sdf).
#   ./scripts/spawn_person.sh <name> <x> <y> [yaw]
# e.g. ./scripts/spawn_person.sh human2 -7 4 0
# Then walk them: ./scripts/launch_human_waypoints.sh person:=human2 params_file:=<yaml>
# (the yaml's top key must be <name>_waypoints, see config/human2_crossing.yaml).
source "$(dirname "$0")/env.sh"
if [ $# -lt 3 ]; then
    echo "Usage: $0 <name> <x> <y> [yaw]" >&2
    exit 1
fi
NAME=$1; X=$2; Y=$3; YAW=${4:-0}
SDF=$(mktemp --suffix=.sdf)
trap 'rm -f "$SDF"' EXIT
sed "s/PERSON_NAME/$NAME/g" "$(ros2 pkg prefix people_follower)/share/people_follower/models/person.sdf" > "$SDF"
ros2 run ros_gz_sim create -file "$SDF" -name "$NAME" -x "$X" -y "$Y" -z 0 -Y "$YAW"
