# CS485 Project: People-Following Robot

ROS 2 Jazzy + Gazebo Harmonic + RViz2, simulated TurtleBot 4.

## Setup (once per machine)

```bash
sudo apt install ros-jazzy-turtlebot4-simulator ros-jazzy-turtlebot4-desktop ros-jazzy-teleop-twist-keyboard
git clone https://github.com/notBjorn/CS485-Project-People-Following-Robot.git CS485-Project
cd CS485-Project
./scripts/build.sh
```

## Running

```bash
./scripts/launch_sim.sh                     # box_room world + TurtleBot 4 + RViz
./scripts/launch_sim.sh world:=warehouse    # stock TB4 worlds: warehouse, maze, depot
./scripts/launch_sim.sh world:=simple_colored_warehouse   # 30 m x 24 m warehouse with colored boxes
./scripts/launch_sim.sh rviz:=false x:=1.0 y:=-2.0
./scripts/launch_teleop.sh                  # keyboard driving, in a second terminal
./scripts/launch_slam.sh                    # build a map, then ./scripts/save_map.sh <world>
./scripts/launch_nav.sh                     # AMCL + Nav2 on the saved map
./scripts/launch_nav_node.sh                # automated navigation node (with launch_nav.sh)
```

See RUN_DIRECTIONS.txt for step-by-step instructions.

Rebuild with `./scripts/build.sh` after adding files or entry points
(`--symlink-install` means edits to existing Python files apply without rebuilding).

## Scripts

All scripts are in `scripts/` and can be run from anywhere. Arguments in `[brackets]` are optional.

### Build

| Script | What it does |
|---|---|
| `build.sh` | Installs missing dependencies and builds the workspace. Run after adding files, nodes or maps. |
| `clean_build.sh` | Deletes `build/`, `install/` and `log/`, then rebuilds from scratch. Use when a normal build acts stale (e.g. after renaming or deleting files). |

### Simulation

| Script | What it does |
|---|---|
| `launch_sim.sh [args]` | Gazebo + TurtleBot 4 + RViz. Takes `world:=`, `rviz:=`, `x:=`/`y:=`/`yaw:=`, `slam:=`, `localization:=`, `nav2:=`, `map:=`. |
| `launch_teleop.sh` | Drive the robot with the keyboard (run in its own terminal and keep it focused). |
| `list_worlds.sh` | Lists every world you can launch, whether it has a saved map, and which ones download models on first run. |
| `kill_sim.sh` | Stops all leftover Gazebo, bridge, RViz, Nav2 and node processes. Run when Ctrl+C left something behind and the next launch misbehaves. Also stops any other ROS 2 nodes on the machine. |

### Mapping

| Script | What it does |
|---|---|
| `make_map.sh [world]` | Guided mapping in one command: starts SLAM, opens teleop in a new window, saves the map when you press Enter, then rebuilds. The map is named after the world (default `box_room`). |
| `launch_slam.sh [args]` | Simulation with SLAM only, for mapping by hand. |
| `save_map.sh [name]` | Saves the current SLAM map to `src/people_follower/maps/<name>.yaml/.pgm`. Name it after the world, then run `build.sh`. |

### Navigation

| Script | What it does |
|---|---|
| `nav_demo.sh [sim args] [-- node args]` | Automated navigation in one command: starts the sim with localization and Nav2, waits until Nav2 is active, then runs `nav_automation_node`. Ctrl+C stops everything. Launch output goes to `log/nav_demo_launch.log`. Example: `./scripts/nav_demo.sh world:=warehouse -- -p goal_x:=3.0 -p goal_y:=-2.0` |
| `launch_nav.sh [args]` | Simulation with AMCL localization and Nav2 on the world's saved map. |
| `launch_nav_node.sh [-p param:=value ...]` | Runs `nav_automation_node` against an already-running `launch_nav.sh`. Parameters: `initial_x`, `initial_y`, `initial_direction`, `goal_x`, `goal_y`, `update_period`, `resend_distance`. |
| `set_initial_pose.sh [x] [y] [direction]` | Tells AMCL where the robot is, like RViz's "2D Pose Estimate". Defaults to `0 0 0` (the default spawn). Direction is in degrees; 0 faces +x. |
| `send_goal.sh <x> <y> [direction]` | Sends one Nav2 goal from the terminal and prints progress. Useful for checking Nav2 works on its own before testing the node. |

### Testing

| Script | What it does |
|---|---|
| `check_sim.sh` | Health check for a running sim: which sensor topics exist and their rates, whether the robot is localized on the map, and whether each navigation node is active. Run it when "nothing happens". |
| `record_bag.sh [name] [--no-camera]` | Records the LiDAR, camera, odometry, TF, drive commands and navigation topics to `bags/<name>_<time>/` (not committed). Ctrl+C stops recording. Replay with `ros2 bag play bags/<folder> --clock`. Use `--no-camera` for much smaller files. |

`env.sh` isn't run directly: the other scripts source it to set up ROS 2 and the workspace.
Run `source scripts/env.sh` in a terminal to use `ros2` commands by hand.

## Layout

```
src/people_follower/          ament_python package (our code)
  launch/sim.launch.py        Gazebo + TB4 spawn + RViz
  worlds/box_room.sdf         empty 10 m x 10 m walled room (M1)
  worlds/simple_colored_warehouse.sdf   30 m x 24 m warehouse, by Arshal on Gazebo Fuel (CC BY 4.0)
  maps/                       saved SLAM maps (<world>.yaml + .pgm)
  people_follower/            Python nodes go here; register them in setup.py
  config/                     parameter files
src/turtlebot4_waypoint_auto_nav/   automated Nav2 navigation node (nav_automation_node)
scripts/                      helper scripts (see Scripts above)
```

## Useful topics

| Topic | Type | Notes |
|---|---|---|
| `/scan` | sensor_msgs/LaserScan | RPLIDAR |
| `/oakd/rgb/preview/image_raw` | sensor_msgs/Image | OAK-D RGB |
| `/oakd/rgb/preview/depth` | sensor_msgs/Image | depth |
| `/cmd_vel` | geometry_msgs/TwistStamped | drive the robot (stamped) |
| `/odom` | nav_msgs/Odometry | |

`ROS_DOMAIN_ID` defaults to 30 and discovery is localhost-only (see `scripts/env.sh`).
