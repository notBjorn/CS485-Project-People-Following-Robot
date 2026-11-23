#!/usr/bin/env bash
# Remove Fast DDS shared-memory files in /dev/shm that no running process has open.
# launch and kill_sim scripts run this for you; running processes' files are left alone.
#
# A ROS node killed at the wrong moment can leave a port's mutex locked for good. Every
# later launch then has a node that picks that port and waits forever: no clock, no robot,
# and "ros2" commands that hang. "fastdds shm clean" doesn't remove these.
in_use=$( { cat /proc/[0-9]*/maps; ls -l /proc/[0-9]*/fd; } 2>/dev/null |
          grep -o '/dev/shm/[^ ]*fastrtps[^ ]*' | sort -u )
for f in /dev/shm/fastrtps* /dev/shm/sem.fastrtps*; do
    [ -e "$f" ] || continue
    grep -qxF "$f" <<<"$in_use" || rm -f "$f"
done
exit 0
