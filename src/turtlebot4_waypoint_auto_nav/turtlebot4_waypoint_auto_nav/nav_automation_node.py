"""Dock, set the initial pose, undock, then keep Nav2 heading toward a target.

Needs localization + Nav2 running on a saved map, e.g.
    ./scripts/launch_sim.sh localization:=true nav2:=true
then
    ./scripts/launch_nav_node.sh

The target is a fixed goal for now (goal_x / goal_y parameters). For person
following, replace get_target() with the latest person position from the
perception node: the loop already re-sends the goal whenever the target moves.
"""

import math
import time

import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def get_target(navigator):
    """Return the [x, y] (map frame, metres) the robot should drive toward, or None."""
    # TODO: replace with the person's position from the perception node,
    # offset back by the following distance.
    return [navigator.get_parameter('goal_x').value,
            navigator.get_parameter('goal_y').value]


def main():
    rclpy.init()
    navigator = TurtleBot4Navigator()

    # Defaults match the default spawn pose in sim.launch.py (x=0, y=0, yaw=0).
    navigator.declare_parameter('initial_x', 0.0)
    navigator.declare_parameter('initial_y', 0.0)
    navigator.declare_parameter('initial_direction', float(TurtleBot4Directions.NORTH))
    navigator.declare_parameter('goal_x', 2.0)  # inside box_room (walls at +/-5 m)
    navigator.declare_parameter('goal_y', 1.0)
    navigator.declare_parameter('update_period', 1.0)   # seconds between target checks
    navigator.declare_parameter('resend_distance', 0.3)  # metres the target must move to re-send

    # 1. Dock first so the robot starts from a known spot
    if not navigator.getDockedStatus():
        navigator.info('Docking before initializing pose')
        navigator.dock()

    # 2. Tell AMCL where the robot is
    initial_pose = navigator.getPoseStamped(
        [navigator.get_parameter('initial_x').value, navigator.get_parameter('initial_y').value],
        navigator.get_parameter('initial_direction').value)
    navigator.setInitialPose(initial_pose)

    # 3. Wait for Nav2, then undock
    navigator.waitUntilNav2Active()
    navigator.undock()

    # 4. Keep driving toward the target. goToPose() returns immediately, and
    # sending a new goal replaces the old one, so the goal can follow a moving target.
    update_period = navigator.get_parameter('update_period').value
    resend_distance = navigator.get_parameter('resend_distance').value
    last_goal = None
    try:
        while rclpy.ok():
            target = get_target(navigator)
            if target is not None and (
                    last_goal is None or math.dist(target, last_goal) > resend_distance):
                navigator.info(f'New goal: x={target[0]:.2f}, y={target[1]:.2f}')
                navigator.goToPose(navigator.getPoseStamped(target, TurtleBot4Directions.NORTH))
                last_goal = target
            # Wait update_period while still handling callbacks (e.g. a future person subscriber)
            deadline = time.monotonic() + update_period
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(navigator, timeout_sec=max(0.0, deadline - time.monotonic()))
    except KeyboardInterrupt:
        navigator.cancelTask()
    finally:
        navigator.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
