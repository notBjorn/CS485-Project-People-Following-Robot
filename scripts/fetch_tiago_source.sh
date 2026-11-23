#!/usr/bin/env bash
# Fallback for when ./scripts/build.sh can't install the TIAGo model with apt/rosdep
# (ros-jazzy-easynav-playground-tiago-description / -worlds): clones the EasyNav
# playgrounds into src/external/ (not committed) and keeps only the two TIAGo
# packages we use, then builds. Run it once per machine; re-run to update.
set -e
cd "$(dirname "$0")/.."
DEST=src/external/easynav_playgrounds
KEEP=(easynav_playground_tiago_description easynav_playground_tiago_worlds)

if [ -d "$DEST/.git" ]; then
    git -C "$DEST" pull --ff-only
else
    mkdir -p src/external
    git clone --depth 1 -b jazzy https://github.com/EasyNavigation/easynav_playgrounds.git "$DEST"
fi

# colcon skips folders with a COLCON_IGNORE file: ignore every other playground package
# (they need EasyNav, which we don't use)
find "$DEST" -name package.xml | while read -r xml; do
    dir=$(dirname "$xml")
    if [[ " ${KEEP[*]} " == *" $(basename "$dir") "* ]]; then rm -f "$dir/COLCON_IGNORE"
    else touch "$dir/COLCON_IGNORE"; fi
done

./scripts/build.sh "$@"
