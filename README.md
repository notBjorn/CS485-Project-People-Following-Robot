# CS485 Project: People-Following Robot

A simulated TurtleBot 4 that follows a person using UWB (ultra-wideband) ranging,
with ROS 2 Jazzy, Gazebo Harmonic and RViz2.

## Requirements

- Ubuntu 24.04 with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) (desktop install)
- An internet connection for setup and the first launch of some worlds (models download from Gazebo Fuel)
- A GPU is recommended for Gazebo's rendering and simulated LiDAR

## Setup (once per machine)

1. Install the TurtleBot 4 simulator:

   ```bash
   sudo apt install ros-jazzy-turtlebot4-simulator ros-jazzy-turtlebot4-desktop \
       ros-jazzy-teleop-twist-keyboard
   ```

2. Get the code:

   ```bash
   git clone https://github.com/notBjorn/CS485-Project-People-Following-Robot.git CS485-Project
   cd CS485-Project
   ```

3. Build the workspace. This also installs any missing ROS dependencies and builds the
   UWB Gazebo plugin (`src/uwb_gazebo_plugin`):

   ```bash
   ./scripts/build.sh
   ```

4. Check it works: this should open Gazebo with the TurtleBot 4 and RViz.

   ```bash
   ./scripts/launch_sim.sh
   ```

Rebuild with `./scripts/build.sh` after pulling changes that add files. Edits to
existing Python files apply without rebuilding.

## Network

The scripts set `ROS_DOMAIN_ID` to 30 (see `scripts/env.sh`), and ROS 2 discovery
reaches your local network. On a shared network, give each person their own ID before
launching, so your simulations don't see each other:

```bash
export ROS_DOMAIN_ID=<0-101>
```
