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
./scripts/launch_sim.sh rviz:=false x:=1.0 y:=-2.0
./scripts/launch_teleop.sh                  # keyboard driving, in a second terminal
```

Rebuild with `./scripts/build.sh` after adding files or entry points
(`--symlink-install` means edits to existing Python files apply without rebuilding).

## Layout

```
src/people_follower/          ament_python package (our code)
  launch/sim.launch.py        Gazebo + TB4 spawn + RViz
  worlds/box_room.sdf         empty 10 m x 10 m walled room (M1)
  people_follower/            Python nodes go here; register them in setup.py
  config/                     parameter files
scripts/                      env.sh (sourced by the others), build, sim, teleop
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
