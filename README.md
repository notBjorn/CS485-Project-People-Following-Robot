# CS485 Project: People-Following Robot

A simulated robot that follows a person using UWB (ultra-wideband) ranging, with
ROS 2 Jazzy, Gazebo Harmonic and RViz2. The robot is PAL's TIAGo by default, or the
TurtleBot 4 with `robot:=turtlebot4` on any launch, demo or mapping script.

TIAGo is the Gazebo Harmonic port from the
[EasyNav playgrounds](https://github.com/EasyNavigation/easynav_playgrounds)
(`easynav_playground_tiago_description` / `_worlds`; EasyNav itself isn't used). PAL's
own TIAGo simulation only supports ROS 2 Humble with Gazebo Classic.

## Requirements

- Ubuntu 24.04 with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) (desktop install)
- An internet connection for setup and the first launch of some worlds (models download from Gazebo Fuel)
- A GPU is recommended for Gazebo's rendering and simulated LiDAR

## Setup (once per machine)

1. Install the TurtleBot 4 simulator (also needed for TIAGo runs, which share its tools):

   ```bash
   sudo apt install ros-jazzy-turtlebot4-simulator ros-jazzy-turtlebot4-desktop \
       ros-jazzy-teleop-twist-keyboard
   ```

2. Get the code:

   ```bash
   git clone https://github.com/notBjorn/CS485-Project-People-Following-Robot.git CS485-Project
   cd CS485-Project
   ```

3. Build the workspace. This also installs any missing ROS dependencies (including
   TIAGo's model, Nav2 and slam_toolbox) and builds the UWB Gazebo plugin
   (`src/uwb_gazebo_plugin`):

   ```bash
   ./scripts/build.sh
   ```

   If it ends with a warning that TIAGo's model is not installed, run
   `sudo apt install ros-jazzy-easynav-playground-tiago-worlds`, or build it from
   source with `./scripts/fetch_tiago_source.sh`.

4. Check it works: this should open Gazebo with TIAGo and RViz.

   ```bash
   ./scripts/launch_sim.sh                       # robot:=turtlebot4 for the TurtleBot 4
   ```

Rebuild with `./scripts/build.sh` after pulling changes that add files. Edits to
existing Python files apply without rebuilding.

## TIAGo and the TurtleBot 4

Both robots carry UWB anchors 1-4 and look the same to the rest of the project:
drive on `/cmd_vel` (TwistStamped), odometry on `/odom`, laser on `/scan`. On TIAGo,
`tiago_base_relay` connects those to its `mobile_base_controller`. The differences:

| | TIAGo | TurtleBot 4 |
|---|---|---|
| Camera | `/head_front_camera/rgb/image_raw` | `/oakd/rgb/preview/image_raw` |
| UWB anchors | base, 0.30 m apart (`urdf/tiago_uwb.urdf.xacro`) | top plate, 0.17 m apart |
| Follow distance (`launch_follow.sh`) | 0.88 m centre to person: edge stops 2 ft (0.61 m) away; slows within 6 ft | 0.79 m: same 2 ft / 6 ft |
| Top following speed (`max_linear`) | 1.0 m/s (the base's limit) | 0.3 m/s |
| Dock | none: nothing to undock | starts docked; scripts undock it |
| SLAM / Nav2 config | stock Jazzy + `config/tiago/*_overrides.yaml` | TurtleBot 4's own |

`launch_follow.sh`, `auto_map.sh`, `make_map.sh` and `nav_automation_node` check which
robot is running. See `How-To-Run/Master_Script_List.md` for every script.

## Network

The scripts set `ROS_DOMAIN_ID` to 30 (see `scripts/env.sh`), and ROS 2 discovery
reaches your local network. On a shared network, give each person their own ID before
launching, so your simulations don't see each other:

```bash
export ROS_DOMAIN_ID=<0-101>
```
