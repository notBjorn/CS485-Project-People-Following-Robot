# Master Script List

Every shell script in `scripts/`. Run them from the project folder, e.g.
`./scripts/open_box_room.sh`. Arguments in `[brackets]` are optional; Ctrl+C stops a
script and everything it started.

## Setup and building

| Script | What it does |
|---|---|
| `setup_yolo.sh` | One-time setup for person detection: creates the `.venv-yolo` Python environment with YOLO (about 3 GB) and downloads the model into `models/`. |
| `build.sh` | Installs missing ROS dependencies and builds the workspace. Run it after pulling changes that add files, nodes or maps. |
| `clean_build.sh` | Deletes `build/`, `install/` and `log/`, then builds from scratch. Use it when a normal build behaves as if it's out of date. |

## Opening a world

Each opens Gazebo with the TurtleBot 4 and RViz in one world. Extra settings pass through, e.g. `rviz:=false`.

| Script | What it does |
|---|---|
| `open_box_room.sh` | Opens `box_room`, an empty 10 m x 10 m walled room (the default world). |
| `open_person_world.sh` | Opens `person_world`, a 25 m x 25 m walled square with one person to follow. |
| `open_simple_colored_warehouse.sh` | Opens `simple_colored_warehouse`, a 30 m x 24 m warehouse with four rooms, coloured boxes and one person. |
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
| `spawn_person.sh <name> <x> <y> [yaw]` | Adds another person to the running simulation, e.g. `human2 -7 4 0`. |

## Mapping

| Script | What it does |
|---|---|
| `auto_map.sh [world] [map name]` | The robot maps the world by itself: it explores until no unmapped edges are left, then saves the map and rebuilds. Watch the map grow in RViz. |
| `make_map.sh [world]` | Maps a world while you drive with the keyboard (it opens a teleop window), then saves the map when you press Enter. |
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

## Following a person (camera)

| Script | What it does |
|---|---|
| `launch_camera_follow.sh [noise:=true]` | Undocks the robot and starts camera following: YOLO detection, depth, tracking, the follower and the safety guard. Start the warehouse with `yaw:=3.14159` first. |
| `launch_person_detector.sh [noise:=true]` | Runs only the camera side: YOLO detection, depth and tracking, without driving the robot. |

## Demos

| Script | What it does |
|---|---|
| `demo_follow.sh [noise:=true]` | Demo 1: the robot follows a person once round the coloured box in each of the warehouse's four rooms. Opens Gazebo and a camera window, and runs until Ctrl+C. |
| `demo_two_people.sh [noise:=true]` | Demo 2: the same in rooms 1 and 2, with a second person crossing the target's path. On Ctrl+C it prints who the robot was following. |

## Tests

| Script | What it does |
|---|---|
| `test_safety.sh [noise:=true]` | Drives the robot at full speed at a wall, the dock and a person through the safety guard, and reports how close it got. About 3 minutes. |
| `test_two_people.sh [noise:=false]` | Short following run with a second person crossing; reports who the robot really followed, the following distance and any bumps. About 6 minutes. |
| `check_sim.sh` | Health check for a running simulation: sensor rates, localization and Nav2 state. Run it when nothing seems to happen. |
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
| `demo_common.sh` | Shared steps for the two demo scripts. |
