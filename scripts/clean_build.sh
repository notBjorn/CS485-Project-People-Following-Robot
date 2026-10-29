#!/usr/bin/env bash
# Delete build/, install/ and log/ and rebuild from scratch.
# Use when a normal build acts stale (e.g. after renaming or deleting files).
set -e
cd "$(dirname "$0")/.."
rm -rf build install log
./scripts/build.sh "$@"
