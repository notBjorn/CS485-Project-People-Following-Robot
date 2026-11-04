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

from nav2_simple_commander.robot_navigator import TaskResult
import rclpy
from rclpy.executors import ExternalShutdownException
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

# Dock geometry, relative to the robot's docked pose, in metres along (forward, left).
# The dock is below the lidar, so Nav2 can't see it and plans straight through it.
DOCK_CENTRE = 0.157         # forward of the docked pose (turtlebot4_spawn.launch.py)
UNDOCKED = -0.36            # where undocking leaves the robot
DOCK_CLEARANCE = 0.45       # go around if the straight path passes closer than this
DETOUR = (-0.7, 0.9)        # point to go around by; the side is chosen to suit the goal


def dock_detour(docked_xy, docked_direction, goal_xy):
    """Return a [x, y] to visit first if driving straight to the goal would hit the dock."""
    yaw = math.radians(docked_direction)
    forward, left = (math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))

    # The goal in the docked pose's frame, so the dock is on the forward axis
    goal_rel = (goal_xy[0] - docked_xy[0], goal_xy[1] - docked_xy[1])
    goal_fwd = goal_rel[0] * forward[0] + goal_rel[1] * forward[1]
    goal_left = goal_rel[0] * left[0] + goal_rel[1] * left[1]

    # Closest the straight path from the undocked spot to the goal comes to the dock
    path = (goal_fwd - UNDOCKED, goal_left)
    length_sq = path[0] ** 2 + path[1] ** 2
    along = 0.0 if length_sq == 0.0 else (DOCK_CENTRE - UNDOCKED) * path[0] / length_sq
    if along <= 0.0:
        return None     # heading away from the dock
    along = min(1.0, along)
    closest = math.hypot(UNDOCKED + along * path[0] - DOCK_CENTRE, along * path[1])
    if closest >= DOCK_CLEARANCE:
        return None

    side = -1.0 if goal_left < 0.0 else 1.0
    return [docked_xy[0] + DETOUR[0] * forward[0] + side * DETOUR[1] * left[0],
            docked_xy[1] + DETOUR[0] * forward[1] + side * DETOUR[1] * left[1]]


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
    leaving_dock = True
    try:
        while rclpy.ok():
            target = get_target(navigator)

            # First goal only: don't drive through the dock, which Nav2 can't see
            if leaving_dock and target is not None:
                leaving_dock = False
                detour = dock_detour([navigator.get_parameter('initial_x').value,
                                      navigator.get_parameter('initial_y').value],
                                     navigator.get_parameter('initial_direction').value, target)
                if detour is not None:
                    navigator.info(f'Going around the dock via x={detour[0]:.2f}, y={detour[1]:.2f}')
                    heading = math.degrees(math.atan2(target[1] - detour[1], target[0] - detour[0]))
                    navigator.goToPose(navigator.getPoseStamped(detour, heading))
                    while rclpy.ok() and not navigator.isTaskComplete():
                        pass

            # A failed goal is never retried otherwise, because the target hasn't moved
            if (last_goal is not None and navigator.isTaskComplete() and
                    navigator.getResult() == TaskResult.FAILED):
                navigator.warn('Goal failed - trying again')
                last_goal = None

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
    except ExternalShutdownException:
        pass
    finally:
        navigator.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
