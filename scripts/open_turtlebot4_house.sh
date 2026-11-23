#!/usr/bin/env bash
# Open the turtlebot4_house world: a detailed, furnished 10 m x 8 m house (living room,
# dining room, bedroom, bathroom; 0.9 m doors) with one person wearing a UWB tag in the
# living room (either robot carries the anchors). Gazebo + robot (TIAGo, or
# robot:=turtlebot4) + RViz. The robot spawns 2.2 m behind the person, as in
# demo_uwb_house.sh. Extra args pass through, e.g. rviz:=false robot:=turtlebot4
exec "$(dirname "$0")/launch_sim.sh" world:=turtlebot4_house x:=0.0 y:=-1.2 yaw:=1.5708 "$@"
