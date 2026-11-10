"""Smooth the camera's person positions over time and pick the person to follow.

Takes the per-frame positions from person_localizer and runs a Kalman filter per
person in the odom frame (state x, y, vx, vy; constant-velocity motion), so the
robot's own driving and turning don't move the estimates.

Each measurement's noise follows the camera: along the line of sight it is the stereo
depth error, which grows with distance squared (range_sigma_base + range_sigma_k * r^2),
while across it is small (bearing_sigma_rad * r), because the box position in the image
gives the direction to about a degree.

Unseen, a track moves on its last velocity for at most max_extrapolation_s, then holds.

Association: all detection-track pairs within gate_sigmas (Mahalanobis distance) are
taken best first, so when two people are close each keeps their own track. A pair whose
YOLO tracking ID matches the one the track had last time counts as a closer fit
(yolo_id_bonus). A detection that fits no track starts a new one. A track is confirmed
after confirm_hits detections, so the odd false box never becomes a person, and is
dropped after track_timeout_s without one (it keeps predicting until then, which carries
the person through short gaps, such as leaving the camera's view or being walked past).

Target:
  - At the start, the person to follow is the closest confirmed person within
    start_radius of the robot: the person standing by it when it starts. Anyone further
    away is ignored (the follower keeps turning to look until someone is that close).
  - The tracker keeps the target while its track exists.
  - When the target is lost, the next target is the confirmed person closest to where
    they were last seen, but only within reacquire_base_m + reacquire_speed * (time
    since lost), at most reacquire_max_m: as far as they could plausibly have walked.
    Someone else further away is never taken instead.

Subscribes /person/detections_3d (vision_msgs/Detection3DArray, base_link).
Publishes:
    /person/tracks           vision_msgs/Detection3DArray in odom, confirmed tracks
                             (id = track number)
    /person/target_odom      geometry_msgs/PointStamped, the target in odom
    /person/target_velocity  geometry_msgs/Vector3Stamped, its velocity in odom (m/s)
    /person/target           geometry_msgs/PointStamped, the target in base_link
    /person/target_marker    visualization_msgs/Marker for RViz
"""

import math

from geometry_msgs.msg import PointStamped, Vector3Stamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from vision_msgs.msg import Detection3D, Detection3DArray
from visualization_msgs.msg import Marker


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Track:
    """One person: Kalman filter in odom."""

    def __init__(self, number, xy, cov, stamp):
        self.number = number
        self.x = np.array([xy[0], xy[1], 0.0, 0.0])
        self.P = np.zeros((4, 4))
        self.P[:2, :2] = cov
        self.P[2, 2] = self.P[3, 3] = 1.0
        self.time = stamp           # time of the state
        self.last_seen = stamp
        self.hits = 1
        self.yolo_id = ''           # YOLO's tracking ID at the last update

    def predict(self, t, accel_var):
        dt = t - self.time
        if dt <= 0:
            return
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        q1 = np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt ** 2]]) * accel_var
        q = np.zeros((4, 4))
        q[np.ix_([0, 2], [0, 2])] = q1
        q[np.ix_([1, 3], [1, 3])] = q1
        self.x = f @ self.x
        self.P = f @ self.P @ f.T + q
        self.time = t

    def position_at(self, t, max_extrapolation):
        """Position extrapolated to time t (at most max_extrapolation s), filter unchanged."""
        return self.x[:2] + self.x[2:] * min(max(0.0, t - self.time), max_extrapolation)

    def mahalanobis2(self, z, r):
        s = self.P[:2, :2] + r
        d = z - self.x[:2]
        return float(d @ np.linalg.solve(s, d))

    def update(self, z, r, t):
        h = np.zeros((2, 4))
        h[0, 0] = h[1, 1] = 1.0
        s = self.P[:2, :2] + r
        k = self.P @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ (z - self.x[:2])
        i_kh = np.eye(4) - k @ h
        self.P = i_kh @ self.P @ i_kh.T + k @ r @ k.T
        self.last_seen = t
        self.hits += 1


class PersonTracker(Node):

    def __init__(self):
        super().__init__('person_tracker')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('range_sigma_base', 0.05)   # m
        # m per m^2: stereo depth noise, z^2 * disparity_sigma / (fx * baseline); 0.1 px,
        # fx 221 px and 0.075 m (config/sensor_noise.yaml) give 0.006.
        self.declare_parameter('range_sigma_k', 0.006)
        self.declare_parameter('bearing_sigma_rad', 0.02)
        self.declare_parameter('accel_sigma', 1.0)          # m/s^2
        self.declare_parameter('gate_sigmas', 3.0)
        self.declare_parameter('confirm_hits', 3)
        self.declare_parameter('track_timeout_s', 2.0)
        # Unseen tracks move on their last velocity for at most this long, then stay put:
        # the velocity estimate isn't good enough to carry them further.
        self.declare_parameter('max_extrapolation_s', 0.5)
        self.declare_parameter('max_speed', 3.0)            # m/s; faster estimates are slowed
        self.declare_parameter('yolo_id_bonus', 0.5)        # multiplies the fit when IDs match
        self.declare_parameter('start_radius', 3.0)         # m from the robot to be picked at start
        self.declare_parameter('reacquire_base_m', 1.5)
        self.declare_parameter('reacquire_speed', 0.5)      # m/s
        self.declare_parameter('reacquire_max_m', 5.0)
        self.declare_parameter('rate_hz', 20.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.base_frame = p('base_frame')
        self.odom_frame = p('odom_frame')
        self.range_base = p('range_sigma_base')
        self.range_k = p('range_sigma_k')
        self.bearing_sigma = p('bearing_sigma_rad')
        self.accel_var = p('accel_sigma') ** 2
        self.gate2 = p('gate_sigmas') ** 2
        self.confirm_hits = p('confirm_hits')
        self.timeout = p('track_timeout_s')
        self.max_extrapolation = p('max_extrapolation_s')
        self.max_speed = p('max_speed')
        self.yolo_bonus = p('yolo_id_bonus')
        self.start_radius = p('start_radius')
        self.reacquire_base = p('reacquire_base_m')
        self.reacquire_speed = p('reacquire_speed')
        self.reacquire_max = p('reacquire_max_m')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tracks = []
        self.next_number = 1
        self.target = None          # Track
        self.target_last_xy = None  # where the target was last seen, in odom
        self.lost_since = None      # when the target was lost
        self.waiting_logged = False

        self.create_subscription(Detection3DArray, '/person/detections_3d', self.on_detections, 10)
        self.tracks_pub = self.create_publisher(Detection3DArray, '/person/tracks', 10)
        self.target_odom_pub = self.create_publisher(PointStamped, '/person/target_odom', 10)
        self.velocity_pub = self.create_publisher(Vector3Stamped, '/person/target_velocity', 10)
        self.target_pub = self.create_publisher(PointStamped, '/person/target', 10)
        self.marker_pub = self.create_publisher(Marker, '/person/target_marker', 10)
        self.create_timer(1.0 / p('rate_hz'), self.step)
        self.get_logger().info('Tracking people from /person/detections_3d in ' + self.odom_frame)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def robot_pose(self, when=None):
        """(x, y, yaw) of base_link in odom at `when` (latest if unavailable), or None."""
        for t in ((when,) if when is not None else ()) + (Time(),):
            try:
                tf = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, t).transform
            except tf2_ros.TransformException:
                continue
            return tf.translation.x, tf.translation.y, yaw_from_quaternion(tf.rotation)
        return None

    def measurement(self, pose, bx, by):
        """Position in odom and its 2x2 covariance, from a point in base_link."""
        rx, ry, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        z = np.array([rx + c * bx - s * by, ry + s * bx + c * by])
        r = math.hypot(bx, by)
        sigma_range = self.range_base + self.range_k * r * r
        sigma_cross = max(self.bearing_sigma * r, 0.02)
        angle = yaw + math.atan2(by, bx)
        rot = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        cov = rot @ np.diag([sigma_range ** 2, sigma_cross ** 2]) @ rot.T
        return z, cov

    def on_detections(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        t = stamp.nanoseconds * 1e-9
        pose = self.robot_pose(stamp)
        if pose is None:
            return
        for track in self.tracks:
            track.predict(t, self.accel_var)
        measured = [self.measurement(pose, d.bbox.center.position.x, d.bbox.center.position.y)
                    for d in msg.detections]
        # Every pair within the gate, best fit first; each track and detection used once.
        pairs = []
        for ti, track in enumerate(self.tracks):
            for di, (z, r) in enumerate(measured):
                d2 = track.mahalanobis2(z, r)
                if d2 < self.gate2:
                    if track.yolo_id and track.yolo_id == msg.detections[di].id:
                        d2 *= self.yolo_bonus
                    pairs.append((d2, ti, di))
        used_tracks, used_dets = set(), set()
        for _, ti, di in sorted(pairs):
            if ti in used_tracks or di in used_dets:
                continue
            used_tracks.add(ti)
            used_dets.add(di)
            track = self.tracks[ti]
            track.update(*measured[di], t)
            track.yolo_id = msg.detections[di].id
            speed = math.hypot(track.x[2], track.x[3])
            if speed > self.max_speed:
                track.x[2:] *= self.max_speed / speed
        for di, (z, r) in enumerate(measured):
            if di not in used_dets:
                track = Track(self.next_number, z, r, t)
                track.yolo_id = msg.detections[di].id
                self.tracks.append(track)
                self.next_number += 1

    def step(self):
        now = self.now_s()
        pose = self.robot_pose()
        if pose is None:
            return
        dropped = [tr for tr in self.tracks if now - tr.last_seen > self.timeout]
        self.tracks = [tr for tr in self.tracks if tr not in dropped]
        if self.target in dropped:
            self.get_logger().info(f'Lost person {self.target.number}')
            self.target = None
            self.lost_since = now
        # Filters stay at their last measurement time (detections arrive late, stamped
        # at capture); positions are extrapolated to now only for publishing.
        confirmed = [tr for tr in self.tracks if tr.hits >= self.confirm_hits]

        if self.target is None:
            self.choose_target(confirmed, pose, now)

        stamp = self.get_clock().now().to_msg()
        tracks_msg = Detection3DArray()
        tracks_msg.header.stamp = stamp
        tracks_msg.header.frame_id = self.odom_frame
        for track in confirmed:
            det = Detection3D()
            det.header = tracks_msg.header
            det.id = str(track.number)
            det.bbox.center.position.x, det.bbox.center.position.y = track.position_at(
                now, self.max_extrapolation).tolist()
            det.bbox.center.orientation.w = 1.0
            tracks_msg.detections.append(det)
        self.tracks_pub.publish(tracks_msg)

        if self.target is None:
            return
        target_xy = self.target.position_at(now, self.max_extrapolation)
        self.target_last_xy = target_xy
        point = PointStamped()
        point.header.stamp, point.header.frame_id = stamp, self.odom_frame
        point.point.x, point.point.y = target_xy.tolist()
        self.target_odom_pub.publish(point)
        velocity = Vector3Stamped()
        velocity.header = point.header
        velocity.vector.x, velocity.vector.y = self.target.x[2:].tolist()
        self.velocity_pub.publish(velocity)
        rx, ry, yaw = pose
        dx, dy = target_xy[0] - rx, target_xy[1] - ry
        base = PointStamped()
        base.header.stamp, base.header.frame_id = stamp, self.base_frame
        base.point.x = math.cos(yaw) * dx + math.sin(yaw) * dy
        base.point.y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        self.target_pub.publish(base)
        self.publish_marker(point)

    def choose_target(self, confirmed, pose, now):
        """Pick the target: near the robot at the start, near where it was lost after."""
        if self.target_last_xy is None:
            ref, radius, why = pose[:2], self.start_radius, 'of the robot'
        else:
            ref = self.target_last_xy
            radius = min(self.reacquire_base + self.reacquire_speed * (now - self.lost_since),
                         self.reacquire_max)
            why = 'of where they were last seen'
        candidates = []
        for track in confirmed:
            d = math.dist(track.position_at(now, self.max_extrapolation), ref)
            if d <= radius:
                candidates.append((d, track))
        if not candidates:
            if not self.waiting_logged:
                self.get_logger().info(f'Waiting for a person within {radius:.1f} m {why}')
                self.waiting_logged = True
            return
        self.target = min(candidates, key=lambda c: c[0])[1]
        self.waiting_logged = False
        x, y = self.target.position_at(now, self.max_extrapolation)
        self.get_logger().info(
            f'Following person {self.target.number} at ({x:.1f}, {y:.1f}) in {self.odom_frame}')

    def publish_marker(self, point):
        m = Marker()
        m.header = point.header
        m.ns, m.id, m.type, m.action = 'person_target', 0, Marker.CYLINDER, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = point.point.x, point.point.y, 0.9
        m.pose.orientation.w = 1.0
        sigma = math.sqrt(max(self.target.P[0, 0], self.target.P[1, 1]))
        m.scale.x = m.scale.y = max(0.5, 4.0 * sigma)
        m.scale.z = 1.8
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.3, 0.5
        self.marker_pub.publish(m)


def main():
    rclpy.init()
    node = PersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
