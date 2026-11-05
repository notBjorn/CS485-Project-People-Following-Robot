#!/usr/bin/env bash
# List launchable worlds and whether each has a saved map for navigation.
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUR_WORLDS="$PROJECT_ROOT/src/people_follower/worlds"
OUR_MAPS="$PROJECT_ROOT/src/people_follower/maps"
TB4_WORLDS=/opt/ros/jazzy/share/turtlebot4_gz_bringup/worlds
TB4_MAPS=/opt/ros/jazzy/share/turtlebot4_navigation/maps

printf "%-12s %-12s %-14s %s\n" WORLD SOURCE MAP NOTES
for dir in "$OUR_WORLDS" "$TB4_WORLDS"; do
    [ "$dir" = "$OUR_WORLDS" ] && src=project || src=turtlebot4
    for f in "$dir"/*.sdf; do
        [ -e "$f" ] || continue
        w=$(basename "$f" .sdf)
        if   [ -f "$OUR_MAPS/$w.yaml" ]; then map="yes (ours)"
        elif [ -f "$TB4_MAPS/$w.yaml" ]; then map="yes (tb4)"
        else map="no"; fi
        notes=""
        grep -q "fuel.gazebosim.org" "$f" && notes="downloads models on first run"
        printf "%-12s %-12s %-14s %s\n" "$w" "$src" "$map" "$notes"
    done
done
echo
echo "Launch with: ./scripts/launch_sim.sh world:=<WORLD>   (maps: ./scripts/make_map.sh <WORLD>)"
