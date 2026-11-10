"""Safety layer: no drive command can make the robot run into anything.

Sits between whatever drives the robot and the robot:
    follower (or anything else) -> /cmd_vel_unsafe -> safety_guard -> /cmd_vel -> robot

Safety zone: the robot's round body (robot_radius) plus stop_margin. Forward (and
backward) speed is limited so that, allowing for reaction time (latency_s) and braking
(max_decel), the robot always stops before anything enters the zone. It slows down
smoothly as it gets close and stops at the zone's edge. Turning on the spot is always
allowed: the base is round, so turning sweeps nothing new.

What counts as an obstacle:
  - LiDAR, all round. Returns at or below the LiDAR's minimum range mean something is
    too close to measure, so they count as touching (Gazebo reports them as range_min).
  - Depth camera (use_depth), ahead only: points between obstacle_min_height and
    obstacle_max_height above the floor. This catches low things the LiDAR (19 cm up)
    misses, such as the dock. The camera can't see the floor within about 0.5 m of the
    robot, so obstacles it saw are remembered in odom for depth_memory_s.

Fail-safe: forward/backward motion stops when the LiDAR goes quiet (scan_timeout_s),
when no command has come in for cmd_timeout_s, and for bump_hold_s after a bump.

Subscribes /cmd_vel_unsafe (TwistStamped), /scan, /hazard_detection, the depth image
and camera_info. Publishes /cmd_vel (TwistStamped) and /safety/zone (Marker: the zone,
red while it is limiting the robot).
"""

import math

from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.msg import HazardDetectionVector
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
import tf2_ros
from visualization_msgs.msg import Marker


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class SafetyGuard(Node):

    def __init__(self):
        super().__init__('safety_guard')
        params = {
            'input_topic': '/cmd_vel_unsafe',
            'output_topic': '/cmd_vel',
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'robot_radius': 0.17,        # m; Create 3 body is 0.164
            'stop_margin': 0.15,         # m of free space kept around the body
            'side_margin': 0.05,         # m; the path is checked this much wider each side
            'max_decel': 0.5,            # m/s^2 braking assumed
            'latency_s': 0.15,           # reaction time: sensing + command + motor
            'scan_timeout_s': 0.3,
            'cmd_timeout_s': 0.5,
            'bump_hold_s': 1.0,
            'use_depth': True,
            'depth_topic': '/oakd/rgb/preview/depth',
            'camera_info_topic': '/oakd/rgb/preview/camera_info',
            'camera_optical_frame': 'oakd_rgb_camera_optical_frame',
            'depth_stride': 4,           # use every n-th pixel
            'obstacle_min_height': 0.05,  # m above the floor
            'obstacle_max_height': 0.5,  # m; the robot is about 0.37 m tall
            'depth_range_m': 2.0,        # ignore depth points further than this
            'depth_memory_s': 3.0,
            'rate_hz': 30.0,
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in params}
        self.p = p
        self.radius = p['robot_radius']
        self.zone = p['robot_radius'] + p['stop_margin']

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()
        self.scan_xy = np.zeros((0, 2))     # LiDAR points in base_link
        self.scan_time = None
        self.scan_tf = None                 # (x, y, yaw) of the LiDAR in base_link
        self.depth_points = np.zeros((0, 3))  # remembered obstacles: odom x, y, time seen
        self.intrinsics = None
        self.cmd = None
        self.cmd_time = None
        self.bump_time = None
        self.limiting = None                # last reason, for logging changes

        self.create_subscription(TwistStamped, p['input_topic'], self.on_cmd, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.on_hazard,
                                 qos_profile_sensor_data)
        if p['use_depth']:
            self.create_subscription(Image, p['depth_topic'], self.on_depth,
                                     qos_profile_sensor_data)
            self.create_subscription(CameraInfo, p['camera_info_topic'], self.on_info,
                                     qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(TwistStamped, p['output_topic'], 10)
        self.zone_pub = self.create_publisher(Marker, '/safety/zone', 10)
        self.create_timer(1.0 / p['rate_hz'], self.step)
        self.get_logger().info(
            f"Safety zone {self.zone:.2f} m from the robot's centre "
            f"({p['stop_margin']:g} m clear of its body); {p['input_topic']} -> "
            f"{p['output_topic']}; depth camera {'on' if p['use_depth'] else 'off'}")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # --- inputs ------------------------------------------------------------------

    def on_cmd(self, msg):
        self.cmd = msg
        self.cmd_time = self.now_s()

    def on_hazard(self, msg):
        if any(h.type == 1 for h in msg.detections):   # BUMP
            self.bump_time = self.now_s()

    def on_info(self, msg):
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_scan(self, msg):
        if self.scan_tf is None:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.p['base_frame'], msg.header.frame_id, Time()).transform
            except tf2_ros.TransformException:
                return
            self.scan_tf = (t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))
        r = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        keep = np.isfinite(r) & (r < msg.range_max)
        # At or below the minimum range: something too close to measure. Count it as being
        # at the minimum range rather than dropping it.
        r = np.where(r <= msg.range_min, msg.range_min, r)
        lx, ly, lyaw = self.scan_tf
        a = angles[keep] + lyaw
        self.scan_xy = np.column_stack((lx + r[keep] * np.cos(a), ly + r[keep] * np.sin(a)))
        self.scan_time = self.now_s()

    def on_depth(self, msg):
        if self.intrinsics is None:
            return
        try:
            cam = self.tf_buffer.lookup_transform(
                self.p['base_frame'], self.p['camera_optical_frame'], Time()).transform
            odom = self.tf_buffer.lookup_transform(
                self.p['odom_frame'], self.p['base_frame'], Time()).transform
        except tf2_ros.TransformException:
            return
        depth = self.bridge.imgmsg_to_cv2(msg)
        if msg.encoding == '16UC1':
            depth = depth.astype(float) / 1000.0
        s = max(1, int(self.p['depth_stride']))
        z = depth[::s, ::s].astype(float)
        rows, cols = np.mgrid[0:depth.shape[0]:s, 0:depth.shape[1]:s]
        fx, fy, cx, cy = self.intrinsics
        valid = np.isfinite(z) & (z > 0.0) & (z < self.p['depth_range_m'])
        z, u, v = z[valid], cols[valid], rows[valid]
        if z.size == 0:
            return
        pts = np.column_stack(((u - cx) / fx * z, (v - cy) / fy * z, z))
        base = pts @ rotation_matrix(cam.rotation).T + np.array(
            [cam.translation.x, cam.translation.y, cam.translation.z])
        h = base[:, 2]
        obstacle = base[(h > self.p['obstacle_min_height']) & (h < self.p['obstacle_max_height'])]
        if obstacle.size == 0:
            return
        yaw = yaw_from_quaternion(odom.rotation)
        c, sn = math.cos(yaw), math.sin(yaw)
        ox = odom.translation.x + c * obstacle[:, 0] - sn * obstacle[:, 1]
        oy = odom.translation.y + sn * obstacle[:, 0] + c * obstacle[:, 1]
        seen = np.column_stack((ox, oy, np.full(len(ox), self.now_s())))
        self.depth_points = np.vstack((self.depth_points, seen))[-20000:]

    # --- the guard ---------------------------------------------------------------

    def remembered_points(self):
        """Depth obstacles seen in the last depth_memory_s, in base_link."""
        now = self.now_s()
        self.depth_points = self.depth_points[now - self.depth_points[:, 2]
                                              < self.p['depth_memory_s']]
        if len(self.depth_points) == 0:
            return np.zeros((0, 2))
        try:
            odom = self.tf_buffer.lookup_transform(
                self.p['odom_frame'], self.p['base_frame'], Time()).transform
        except tf2_ros.TransformException:
            return np.zeros((0, 2))
        yaw = yaw_from_quaternion(odom.rotation)
        dx = self.depth_points[:, 0] - odom.translation.x
        dy = self.depth_points[:, 1] - odom.translation.y
        c, s = math.cos(yaw), math.sin(yaw)
        return np.column_stack((c * dx + s * dy, -s * dx + c * dy))

    def free_distance(self, points, direction):
        """How far the robot can drive straight (direction +1 forward, -1 back) before
        its body touches a point, minus nothing; inf if the path is clear."""
        if len(points) == 0:
            return math.inf
        x = points[:, 0] * direction
        y = np.abs(points[:, 1])
        corridor = self.radius + self.p['side_margin']
        ahead = (x > 0.0) & (y < corridor)
        if not np.any(ahead):
            return math.inf
        x, y = x[ahead], y[ahead]
        # Front of the round body at lateral offset y is sqrt(r^2 - y^2) ahead of the
        # centre; points in the side margin count from the centre line (conservative).
        body = np.sqrt(np.clip(self.radius ** 2 - y ** 2, 0.0, None))
        return float(np.min(x - body))

    def speed_limit(self, free):
        """Fastest speed that still stops stop_margin short, with latency and braking."""
        room = free - self.p['stop_margin']
        if room <= 0.0:
            return 0.0
        a, lat = self.p['max_decel'], self.p['latency_s']
        # v * latency + v^2 / (2 a) = room
        return a * (-lat + math.sqrt(lat * lat + 2.0 * room / a))

    def step(self):
        now = self.now_s()
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.p['base_frame']
        reason = None
        if self.cmd is None or now - self.cmd_time > self.p['cmd_timeout_s']:
            reason = 'no drive command'
        else:
            out.twist = self.cmd.twist
            v = out.twist.linear.x
            if self.scan_time is None or now - self.scan_time > self.p['scan_timeout_s']:
                if v != 0.0:
                    reason = 'LiDAR not reporting: no forward/backward motion'
                out.twist.linear.x = 0.0
            elif v != 0.0:
                points = self.scan_xy
                if v > 0.0 and self.p['use_depth']:
                    points = np.vstack((points, self.remembered_points()))
                direction = 1.0 if v > 0.0 else -1.0
                free = self.free_distance(points, direction)
                limit = self.speed_limit(free)
                if direction > 0 and self.bump_time is not None \
                        and now - self.bump_time < self.p['bump_hold_s']:
                    limit, reason = 0.0, 'bumped: holding'
                if abs(v) > limit:
                    out.twist.linear.x = direction * limit
                    if reason is None:
                        reason = (f'stopped: obstacle {free:.2f} m from the body' if limit == 0.0
                                  else f'slowed to {limit:.2f} m/s: obstacle {free:.2f} m away')
        self.cmd_pub.publish(out)
        self.report(reason, out.header)

    def report(self, reason, header):
        kind = None if reason is None else reason.split(':')[0].split(' to ')[0]
        if kind != self.limiting:
            self.limiting = kind
            self.get_logger().info(reason if reason else 'Clear: commands pass through unchanged')
        m = Marker()
        m.header = header
        m.ns, m.id, m.type, m.action = 'safety_zone', 0, Marker.CYLINDER, Marker.ADD
        m.pose.position.z = 0.01
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 2.0 * self.zone
        m.scale.z = 0.02
        limited = reason is not None and reason != 'no drive command'
        m.color.r, m.color.g, m.color.b, m.color.a = (
            (1.0, 0.2, 0.1, 0.5) if limited else (0.2, 0.8, 0.2, 0.3))
        self.zone_pub.publish(m)


def main():
    rclpy.init()
    node = SafetyGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(TwistStamped())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
