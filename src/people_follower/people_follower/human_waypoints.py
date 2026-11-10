"""Walk the simulated person in person_world through a list of waypoints.

Reads the person's true pose from /model/human/odometry (Gazebo OdometryPublisher)
and steers it with geometry_msgs/Twist on /model/human/cmd_vel (Gazebo
VelocityControl): turn toward the next waypoint, walk forward, repeat.
Run it with scripts/launch_human_waypoints.sh, which also starts the bridges.
Waypoints and speeds are in config/human_waypoints.yaml.
"""

import math
import signal

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class HumanWaypoints(Node):

    def __init__(self):
        super().__init__('human_waypoints')
        # Flat [x1, y1, x2, y2, ...] because ROS parameters can't hold nested lists
        self.declare_parameter('waypoints', [3.0, 3.0, -3.0, 3.0, -3.0, -3.0, 3.0, -3.0])
        self.declare_parameter('speed', 0.3)          # m/s walking speed
        self.declare_parameter('max_turn_rate', 1.0)  # rad/s
        self.declare_parameter('tolerance', 0.2)      # m; how close counts as reached
        self.declare_parameter('loop', True)          # start over after the last waypoint
        self.declare_parameter('pause', 1.0)          # s to stand still at each waypoint

        flat = list(self.get_parameter('waypoints').value)
        if len(flat) < 2 or len(flat) % 2:
            raise ValueError('waypoints must be a flat list of x, y pairs')
        self.waypoints = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
        self.speed = self.get_parameter('speed').value
        self.max_turn = self.get_parameter('max_turn_rate').value
        self.tolerance = self.get_parameter('tolerance').value
        self.loop = self.get_parameter('loop').value
        self.pause = self.get_parameter('pause').value

        self.pose = None          # (x, y, yaw) from odometry
        self.index = 0
        self.pause_until = None   # rclpy Time while standing at a waypoint
        self.done = False

        self.cmd_pub = self.create_publisher(Twist, '/model/human/cmd_vel', 10)
        self.create_subscription(Odometry, '/model/human/odometry', self.on_odom, 10)
        self.create_timer(0.05, self.step)
        route = ' -> '.join(f'({x:g}, {y:g})' for x, y in self.waypoints)
        self.get_logger().info(
            f'{len(self.waypoints)} waypoints, speed {self.speed} m/s, loop={self.loop}: {route}')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def step(self):
        cmd = Twist()
        if self.pose is None:
            self.get_logger().info('Waiting for /model/human/odometry...',
                                   throttle_duration_sec=5.0)
        elif self.done:
            pass
        elif self.pause_until is not None:
            if self.get_clock().now() >= self.pause_until:
                self.pause_until = None
        else:
            x, y, yaw = self.pose
            gx, gy = self.waypoints[self.index]
            distance = math.hypot(gx - x, gy - y)
            if distance < self.tolerance:
                self.get_logger().info(f'Reached waypoint {self.index + 1}: ({gx:g}, {gy:g})')
                self.next_waypoint()
            else:
                error = wrap_angle(math.atan2(gy - y, gx - x) - yaw)
                cmd.angular.z = max(-self.max_turn, min(self.max_turn, 2.0 * error))
                # Turn on the spot when facing well away from the waypoint, then walk
                if abs(error) < math.pi / 3:
                    cmd.linear.x = self.speed * math.cos(error)
        self.cmd_pub.publish(cmd)

    def next_waypoint(self):
        self.index += 1
        if self.index == len(self.waypoints):
            if not self.loop:
                self.done = True
                self.get_logger().info('Last waypoint reached, standing still')
                return
            self.index = 0
        if self.pause > 0:
            self.pause_until = self.get_clock().now() + Duration(seconds=self.pause)


def raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    # Handle Ctrl+C/SIGTERM ourselves instead of rclpy, so the context is still valid
    # for the stop command below. Set explicitly: a process started from a background
    # job inherits SIGINT as ignored, and launch escalates to SIGTERM.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, raise_keyboard_interrupt)
    node = HumanWaypoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Gazebo keeps the last velocity, so stop the person. Under launch the bridge
        # may already be gone; launch_human_waypoints.sh also stops it on the Gazebo side.
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
