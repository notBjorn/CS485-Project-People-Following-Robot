# Master Script List

Every shell script in `scripts/` on this branch (TIAGo_uwb). Run them from the
project folder, e.g. `./scripts/open_box_room.sh`. Arguments in `[brackets]` are
optional; Ctrl+C stops a script and everything it started.

The robot is TIAGo unless you add `robot:=turtlebot4` to a script that opens a world
(`open_*`, `launch_*`, the demos, `nav_demo.sh`; for `auto_map.sh` and `make_map.sh`,
after the world name). Scripts that work with a running sim check which robot it is.

## Building

| Script | What it does |
|---|---|
| `build.sh` | Installs missing ROS dependencies and builds the workspace, including the UWB Gazebo plugin. Run it after pulling changes that add files, nodes or maps. Warns if TIAGo's model is missing. |
| `fetch_tiago_source.sh` | Fallback when apt/rosdep can't install TIAGo's model: clones it into `src/external/` (not committed), keeps the two TIAGo packages, and builds. |
| `clean_build.sh` | Deletes `build/`, `install/` and `log/`, then builds from scratch. Use it when a normal build behaves as if it's out of date. |

## Opening a world

Each opens Gazebo with the robot (TIAGo, or the TurtleBot 4 with `robot:=turtlebot4`; both carry four UWB anchors) and RViz in one world. Extra settings pass through, e.g. `rviz:=false`.

| Script | What it does |
|---|---|
| `open_box_room.sh` | Opens `box_room`, an empty 10 m x 10 m walled room (the default world). |
| `open_person_world.sh` | Opens `person_world`, a 25 m x 25 m walled square with one person. |
| `open_simple_colored_warehouse.sh` | Opens `simple_colored_warehouse`, a 30 m x 24 m warehouse with four rooms and coloured boxes, and one person wearing a UWB tag. |
| `open_warehouse.sh` | Opens the TurtleBot 4's stock `warehouse` world. Its models download the first time. |
| `open_maze.sh` | Opens the TurtleBot 4's stock `maze` world. |
| `open_depot.sh` | Opens the TurtleBot 4's stock `depot` world. Its models download the first time. |
| `launch_sim.sh [settings]` | Opens any world with any settings, e.g. `world:=maze x:=1.0 y:=-2.0`. The `open_*` scripts call this. |
| `list_worlds.sh` | Lists every world you can open and whether it has a saved map for navigation. |

## Driving and moving people

| Script | What it does |
|---|---|
| `launch_teleop.sh` | Drives the robot from the keyboard. Run it in its own terminal and keep that window focused. |
| `launch_human_teleop.sh` | Drives the person from the keyboard, with the same keys as `launch_teleop.sh`. |
| `launch_human_waypoints.sh [route]` | Walks the person along a route: `warehouse`, `warehouse_boxes` or `two_rooms` in the warehouse, or the default square in `person_world`. `person:=<name>` walks another person. |
| `spawn_person.sh <name> <x> <y> [yaw]` | Adds another person (without a UWB tag) to the running simulation, e.g. `human2 -7 4 0`. |

## Following a person (UWB)

| Script | What it does |
|---|---|
| `launch_follow.sh` | Starts UWB following (undocking the TurtleBot 4 first): the localizer calibrates the anchors (keep the person still for a few seconds), then the robot follows the tag wearer's path, 0.7 m behind on TIAGo or 0.5 m on the TurtleBot 4. Start the warehouse with `yaw:=3.14159` first. |

## Demos

| Script | What it does |
|---|---|
| `demo_uwb_follow.sh [robot:=turtlebot4]` | Demo 1: the robot follows the person by UWB once round the coloured box in each of the warehouse's four rooms. Builds the workspace first, opens Gazebo and runs until Ctrl+C. |
| `demo_uwb_two_people.sh [robot:=turtlebot4]` | Demo 2: the same in rooms 1 and 2, with a second person (no tag) crossing the target's path. Builds the workspace first. On Ctrl+C it prints who the robot was following. |

## Mapping

| Script | What it does |
|---|---|
| `auto_map.sh [world] [map name] [settings]` | The robot maps the world by itself: it explores until no unmapped edges are left, then saves the map and rebuilds. Watch the map grow in RViz. |
| `make_map.sh [world] [settings]` | Maps a world while you drive with the keyboard (it opens a teleop window), then saves the map when you press Enter. |
| `launch_slam.sh [settings]` | Opens a world with SLAM running, for mapping by hand. |
| `save_map.sh [name]` | Saves the map SLAM has built so far to `src/people_follower/maps/<name>`. Name it after the world, then run `build.sh`. |

## Navigation

| Script | What it does |
|---|---|
| `nav_demo.sh [settings] [-- node settings]` | Opens a world with localization and Nav2 on its saved map, then runs the automated navigation node, all in one command. |
| `launch_nav.sh [settings]` | Opens a world with AMCL localization and Nav2 on its saved map. |
| `launch_nav_node.sh [-p setting:=value]` | Runs the automated navigation node against a running `launch_nav.sh`. |
| `set_initial_pose.sh [x] [y] [direction]` | Tells the robot where it is on the map, like RViz's "2D Pose Estimate". |
| `send_goal.sh <x> <y> [direction]` | Sends one Nav2 goal from the terminal and prints its progress. |

## Testing

| Script | What it does |
|---|---|
| `check_sim.sh` | Health check for a running simulation: which robot, sensor rates, UWB ranges, (TIAGo) its base relay and controller, localization and Nav2 state. Run it when nothing seems to happen. |
| `record_bag.sh [name] [--no-camera]` | Records the robot's sensors, position and commands to `bags/` for replaying later. |

## Troubleshooting

| Script | What it does |
|---|---|
| `kill_sim.sh` | Stops every leftover simulation process (Gazebo, bridges, RViz, Nav2, our nodes). Run it when Ctrl+C left something behind; it also stops other ROS 2 nodes on the machine. |
| `clean_dds.sh` | Removes stale ROS 2 communication files that can make the next launch hang. The launch scripts run it for you. |

## Helpers (not run directly)

| Script | What it does |
|---|---|
| `env.sh` | Sets up ROS 2 and the workspace for the other scripts. Run `source scripts/env.sh` to use `ros2` commands by hand. |
| `sim_start.sh` | Starts the simulation and waits until the robot is ready; the launch scripts use it. |
| `demo_uwb_common.sh` | Shared steps for the two UWB demo scripts, including building the workspace (from scratch if `install/` holds another branch's packages). |
