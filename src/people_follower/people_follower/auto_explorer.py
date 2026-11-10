"""Explore and map a world on its own: drive to the edges of the known map until none are left.

Runs alongside the simulation with SLAM and Nav2 (scripts/auto_map.sh starts all of it).

Loop:
  1. Frontiers: free cells of SLAM's /map next to unknown cells, grouped into connected
     clusters. Clusters smaller than min_frontier_m, and frontier cells closer than
     clearance_m to a wall (Nav2 won't drive there), are ignored.
  2. Goal: for each cluster, its cell nearest the cluster's middle. The next goal is the
     cluster with the lowest cost = distance from the robot - size_weight * cluster size,
     so big nearby openings come first. Goals near ones that failed are skipped.
  3. Nav2 (NavigateToPose) drives there. While it drives, the goal is dropped and a new
     one picked if its frontier has been mapped, or after goal_timeout_s.
Finished when no frontier is left for done_after_s (exit code 0: map complete), when
the only frontiers left are ones Nav2 couldn't reach (exit code 2: map partial), or after
max_duration_s (also 2). auto_map.sh saves the map in both cases; Ctrl+C exits with 1.
"""

import math
import sys
import time

from action_msgs.msg import GoalStatus
import cv2
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros


class AutoExplorer(Node):

    def __init__(self):
        super().__init__('auto_explorer')
        params = {
            'map_frame': 'map',
            'base_frame': 'base_link',
            'min_frontier_m': 0.4,      # smaller openings are ignored
            # frontier cells this close to a wall are ignored; more than Nav2's inflation
            # radius (0.45 m for the TurtleBot 4), which it won't drive into
            'clearance_m': 0.5,
            'size_weight': 0.5,         # m of distance traded for 1 m of frontier size
            'blacklist_radius_m': 0.6,  # skip goals this close to one that failed
            'goal_timeout_s': 60.0,
            'done_after_s': 10.0,       # no frontiers for this long: finished
            'max_duration_s': 900.0,
            'rate_hz': 1.0,
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in params}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map = None
        self.goal = None            # (x, y) of the current goal
        self.goal_handle = None
        self.goal_sent = None       # wall time the goal was sent
        self.blacklist = []
        self.no_frontier_since = None
        self.start = time.time()
        self.finished = False
        self.complete = False
        self.goals_reached = 0
        self.refusals = {}          # goal -> times Nav2 refused it
        self.retry_after = 0.0      # wall time before which no new goal is sent

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.create_timer(1.0 / self.p['rate_hz'], self.step)
        self.get_logger().info('Exploring: waiting for the map and Nav2...')

    def on_map(self, msg):
        self.map = msg

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.p['map_frame'], self.p['base_frame'], Time()).transform
        except tf2_ros.TransformException:
            return None
        return t.translation.x, t.translation.y

    def frontiers(self):
        """[(goal_x, goal_y, size_m)] for each frontier cluster in the current map."""
        m = self.map
        res = m.info.resolution
        grid = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        free = (grid >= 0) & (grid < 50)
        unknown = grid < 0
        occupied = grid >= 50
        kernel = np.ones((3, 3), np.uint8)
        near_unknown = cv2.dilate(unknown.astype(np.uint8), kernel) > 0
        frontier = free & near_unknown
        r = max(1, int(round(self.p['clearance_m'] / res)))
        disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        near_wall = cv2.dilate(occupied.astype(np.uint8), disc) > 0
        frontier &= ~near_wall
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            frontier.astype(np.uint8), connectivity=8)
        out = []
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        min_cells = self.p['min_frontier_m'] / res
        for i in range(1, count):
            if stats[i, cv2.CC_STAT_AREA] < min_cells:
                continue
            rows, cols = np.nonzero(labels == i)
            cx, cy = centroids[i]
            k = int(np.argmin((cols - cx) ** 2 + (rows - cy) ** 2))
            out.append((ox + (cols[k] + 0.5) * res, oy + (rows[k] + 0.5) * res,
                        stats[i, cv2.CC_STAT_AREA] * res))
        return out

    def pick(self, robot):
        """Best frontier goal, and how many frontiers were skipped as unreachable."""
        best, best_cost, skipped = None, None, 0
        for x, y, size in self.frontiers():
            if any(math.dist((x, y), b) < self.p['blacklist_radius_m'] for b in self.blacklist):
                skipped += 1
                continue
            cost = math.dist(robot, (x, y)) - self.p['size_weight'] * size
            if best_cost is None or cost < best_cost:
                best, best_cost = (x, y), cost
        return best, skipped

    def send(self, xy, robot):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.p['map_frame']
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = xy
        yaw = math.atan2(xy[1] - robot[1], xy[0] - robot[0])
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        goal.pose = pose
        self.goal, self.goal_sent, self.goal_handle = xy, time.time(), None
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(self.on_accepted)
        self.get_logger().info(f'Heading for the frontier at ({xy[0]:.1f}, {xy[1]:.1f})')

    def on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            # Nav2 also refuses while it is still starting: retry before giving up on it.
            goal = self.goal
            self.refusals[goal] = self.refusals.get(goal, 0) + 1
            if self.refusals[goal] >= 3:
                self.get_logger().info('Nav2 refused that goal 3 times; skipping it')
                self.fail()
            else:
                self.get_logger().info('Nav2 refused that goal; trying again shortly')
                self.goal = None
            self.retry_after = time.time() + 3.0
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        status = future.result().status
        if self.goal is None:
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goals_reached += 1
            self.goal = None
        elif status != GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Couldn't reach that frontier; skipping it")
            self.fail()

    def fail(self):
        if self.goal is not None:
            self.blacklist.append(self.goal)
        self.goal = None

    def cancel(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal = None

    def step(self):
        if self.finished:
            return
        if time.time() - self.start > self.p['max_duration_s']:
            self.get_logger().info('Time limit reached; stopping exploration')
            self.finish()
            return
        robot = self.robot_xy()
        if (self.map is None or robot is None or not self.nav.server_is_ready()
                or time.time() < self.retry_after):
            return
        if self.goal is not None:
            # Drop the goal if it has been mapped meanwhile or is taking too long.
            still_there = any(math.dist(self.goal, (x, y)) < 1.0 for x, y, _ in self.frontiers())
            if not still_there:
                self.cancel()
            elif time.time() - self.goal_sent > self.p['goal_timeout_s']:
                self.get_logger().info('Taking too long to reach that frontier; skipping it')
                self.blacklist.append(self.goal)
                self.cancel()
            else:
                return
        target, skipped = self.pick(robot)
        if target is None:
            if self.no_frontier_since is None:
                self.no_frontier_since = time.time()
                self.get_logger().info('No reachable frontiers left; checking...')
            elif time.time() - self.no_frontier_since > self.p['done_after_s']:
                if skipped:
                    self.get_logger().warn(
                        f'Stopping with {skipped} unexplored edge(s) Nav2 could not reach: '
                        f'the map is partial ({self.goals_reached} frontiers visited)')
                else:
                    self.complete = True
                    self.get_logger().info(
                        f'Exploration complete: no unexplored edges left '
                        f'({self.goals_reached} frontiers visited)')
                self.finish()
            return
        self.no_frontier_since = None
        self.send(target, robot)

    def finish(self):
        self.cancel()
        self.finished = True


def main():
    """Exit code 0: map complete; 2: stopped with parts unexplored; 1: interrupted."""
    rclpy.init()
    node = AutoExplorer()
    finished = False
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        finished = node.finished
    except KeyboardInterrupt:
        node.cancel()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(1 if not finished else 0 if node.complete else 2)


if __name__ == '__main__':
    main()
