#!/usr/bin/env python3
"""Drive the robot straight at obstacles through safety_guard and measure what's left.

Used by test_safety.sh. The robot (simple_colored_warehouse, spawned at the origin
facing away from the dock, then undocked) is pointed at each target in turn, using
Gazebo's true pose, and sent full speed ahead on /cmd_vel_unsafe for drive_s seconds.
For each run it reports the closest the robot's body came to the target's surface
(Gazebo's true poses), where it stopped, and any bumps.

Targets:
  wall   the end face of mid_wall_right (x = 2, 20 cm wide)
  dock   the charging dock, 10 cm tall: below the LiDAR, only the depth camera sees it
  person the person standing at (2.5, 1.5)
"""

import math
import os
import sys
import time

from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.msg import HazardDetectionVector
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from follow_test import GazeboPoses  # noqa: E402

BODY = 0.164                     # Create 3 body radius
WALL = (2.0, 15.0, -0.1, 0.1)    # mid_wall_right: x min, x max, y min, y max
# Dock collision box: dock at (-0.157, 0) facing +x, box 4 x 13 cm centred 4.5 cm behind it.
DOCK = (-0.157 - 0.045 - 0.02, -0.157 - 0.045 + 0.02, -0.065, 0.065)
PERSON_RADIUS = 0.25


def box_distance(px, py, box):
    x0, x1, y0, y1 = box
    return math.hypot(max(x0 - px, 0.0, px - x1), max(y0 - py, 0.0, py - y1))


class SafetyTest(Node):

    def __init__(self):
        super().__init__('safety_test', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.gz = GazeboPoses('simple_colored_warehouse', ['turtlebot4', 'human'])
        self.cmd = self.create_publisher(TwistStamped, '/cmd_vel_unsafe', 10)
        self.bumps = 0
        self.bumping = False
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.on_hazard,
                                 qos_profile_sensor_data)

    def on_hazard(self, msg):
        bump = any(h.type == 1 for h in msg.detections)
        if bump and not self.bumping:
            self.bumps += 1
        self.bumping = bump

    def send(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.angular.z = linear, angular
        self.cmd.publish(msg)

    def spin_for(self, seconds, linear=0.0, angular=0.0, each=None):
        end = time.time() + seconds
        while time.time() < end:
            self.send(linear, angular)
            rclpy.spin_once(self, timeout_sec=0.03)
            if each:
                each()

    def face(self, x, y, timeout=30.0):
        """Turn on the spot until the robot faces (x, y), using Gazebo's true pose."""
        end = time.time() + timeout
        while time.time() < end:
            r = self.gz.at('turtlebot4')
            if r is None:
                rclpy.spin_once(self, timeout_sec=0.05)
                continue
            err = math.atan2(math.sin(math.atan2(y - r[1], x - r[0]) - r[2]),
                             math.cos(math.atan2(y - r[1], x - r[0]) - r[2]))
            if abs(err) < 0.03:
                break
            self.send(0.0, max(-0.6, min(0.6, 1.5 * err)))
            rclpy.spin_once(self, timeout_sec=0.03)
        self.spin_for(1.0)

    def run(self, name, aim, clearance_of, drive_s):
        self.face(*aim)
        bumps_before = self.bumps
        closest = [math.inf]

        def track():
            r = self.gz.at('turtlebot4')
            if r:
                closest[0] = min(closest[0], clearance_of(r[0], r[1]))
        self.spin_for(drive_s, linear=0.3, each=track)
        self.spin_for(1.5, each=track)
        r = self.gz.at('turtlebot4')
        print(f'{name:7s}: closest the body came {closest[0]:.3f} m; stopped at '
              f'({r[0]:.2f}, {r[1]:.2f}) with {clearance_of(r[0], r[1]):.3f} m left; '
              f'bumps {self.bumps - bumps_before}', flush=True)
        return closest[0], self.bumps - bumps_before


def main():
    rclpy.init()
    node = SafetyTest()
    node.spin_for(3.0)
    if node.gz.at('turtlebot4') is None:
        print('No Gazebo poses; is the simulation running?')
        sys.exit(1)
    results = []
    results.append(node.run(
        'wall', (2.0, 0.0), lambda x, y: box_distance(x, y, WALL) - BODY, drive_s=10.0))
    # Back away from the wall first, so the dock is far enough ahead for the camera.
    node.face(1.0, 0.0)
    node.spin_for(2.0, linear=0.2)
    results.append(node.run(
        'dock', (-0.2, 0.0), lambda x, y: box_distance(x, y, DOCK) - BODY, drive_s=12.0))

    def person_clearance(x, y):
        h = node.gz.at('human')
        return math.dist((x, y), h[:2]) - PERSON_RADIUS - BODY
    h = node.gz.at('human')
    results.append(node.run('person', h[:2], person_clearance, drive_s=15.0))
    worst = min(c for c, _ in results)
    print(f'\nWorst clearance {worst:.3f} m, bumps {sum(b for _, b in results)}: '
          + ('PASS' if worst > 0.05 and sum(b for _, b in results) == 0 else 'FAIL'))
    node.gz.stop()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
