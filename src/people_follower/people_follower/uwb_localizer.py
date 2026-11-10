"""Track the person from UWB ranges: anchors 1-4 on the robot, tag 1 on the person.

Inputs (from src/uwb_gazebo_plugin, bridged by sim.launch.py, std_msgs/Float64 in cm):
    /uwb_gz_simulator/distances/a<I>t<J>               measured range, anchor I to tag J
    /uwb_gz_simulator/distances_ground_truth/a<I>t<J>  true range (calibration and checking only)
Anchor positions come from TF (base_link -> uwb_anchor_<I>, see urdf/turtlebot4_uwb.urdf.xacro).

How it works:
- Calibration. Each anchor reads short by its own fixed offset (antenna delay), like
  real DWM1000 radios. The anchors are only 17-24 cm apart, so offsets that differ by
  10-20 cm would wreck the direction to the person. A real setup measures them once;
  here, with calibrate_with_ground_truth, the node averages (true - measured) for each
  anchor over the first calibration_s seconds, while the person stands still. Otherwise
  it uses the range_offsets_m parameter.
- Tracking. An extended Kalman filter in the odom frame, state [x, y, vx, vy] of the
  person, constant-velocity motion model. Each fresh range is one measurement:
  range = sqrt(horizontal distance^2 + (tag height - anchor height)^2), since the
  anchors are all at one height and can't measure the tag's height themselves.
  Ranges far from the prediction (outliers) are skipped. Working in odom means the
  robot's own driving and turning doesn't blur the estimate.
- Start-up and recovery: a least-squares fix from all anchors starts the filter, and
  restarts it if no range has been accepted for reset_after_s.

Outputs:
    /uwb/person        geometry_msgs/PointStamped, person in base_link (for the follower)
    /uwb/person_odom   geometry_msgs/PointStamped, person in odom
    /uwb/person_velocity geometry_msgs/Vector3Stamped, person's velocity in odom (m/s)
    /uwb/person_truth  geometry_msgs/PointStamped, true person in base_link, solved from the
                       true ranges (simulation only, for checking accuracy)
    /uwb/person_marker visualization_msgs/Marker for RViz
"""

import math

from geometry_msgs.msg import PointStamped, Vector3Stamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
import tf2_ros
from visualization_msgs.msg import Marker


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def solve_position(anchors_xy, horizontal_ranges, guess=None, iterations=8):
    """Least-squares 2D position from >= 3 anchors and horizontal ranges.

    Starts from a linear solve (differences of the circle equations) unless a guess is
    given, then refines with Gauss-Newton. Returns the position, or None.
    """
    a = np.asarray(anchors_xy, dtype=float)
    r = np.asarray(horizontal_ranges, dtype=float)
    if len(a) < 3:
        return None
    if guess is None:
        lhs = 2.0 * (a[1:] - a[0])
        rhs = (r[0] ** 2 - r[1:] ** 2) + np.sum(a[1:] ** 2, axis=1) - np.sum(a[0] ** 2)
        guess, *_ = np.linalg.lstsq(lhs, rhs, rcond=None)
    p = np.array(guess, dtype=float)
    for _ in range(iterations):
        diff = p - a
        dist = np.maximum(np.linalg.norm(diff, axis=1), 1e-6)
        jac = diff / dist[:, None]
        step, *_ = np.linalg.lstsq(jac, r - dist, rcond=None)
        p = p + step
        if np.linalg.norm(step) < 1e-4:
            break
    return p


class UwbLocalizer(Node):

    def __init__(self):
        super().__init__('uwb_localizer')
        self.declare_parameter('anchor_ids', [1, 2, 3, 4])
        self.declare_parameter('tag_id', 1)
        self.declare_parameter('range_topic', '/uwb_gz_simulator/distances')
        self.declare_parameter('truth_topic', '/uwb_gz_simulator/distances_ground_truth')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('tag_height_m', 1.0)        # tag above the floor
        self.declare_parameter('calibrate_with_ground_truth', True)
        self.declare_parameter('calibration_s', 3.0)
        self.declare_parameter('range_offsets_m', [0.22, 0.22, 0.22, 0.22])  # added to each range
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('range_sigma_m', 0.12)      # noise of one range after calibration
        # m/s^2; how quickly the person may change velocity
        # (walking people change speed gently; lower is smoother but lags more at turns)
        self.declare_parameter('accel_sigma', 0.8)
        self.declare_parameter('max_speed', 3.0)           # m/s; faster estimates are slowed
        # Skip ranges further than this many standard deviations from the prediction
        self.declare_parameter('gate_sigmas', 3.0)
        self.declare_parameter('max_range_age_s', 0.3)     # older ranges aren't used for a fix
        self.declare_parameter('max_fix_distance', 20.0)   # m; farther fixes are rejected
        self.declare_parameter('max_fix_rms', 0.4)         # m; worse-fitting fixes are rejected
        self.declare_parameter('reset_after_s', 0.5)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.anchor_ids = list(p('anchor_ids'))
        tag = p('tag_id')
        self.base_frame = p('base_frame')
        self.odom_frame = p('odom_frame')
        self.tag_height = p('tag_height_m')
        self.calibrate = p('calibrate_with_ground_truth')
        self.calibration_s = p('calibration_s')
        self.offsets = dict(zip(self.anchor_ids, p('range_offsets_m')))
        self.range_var = p('range_sigma_m') ** 2
        self.accel_var = p('accel_sigma') ** 2
        self.max_speed = p('max_speed')
        self.max_fix_distance = p('max_fix_distance')
        self.max_fix_rms = p('max_fix_rms')
        self.gate = p('gate_sigmas')
        self.max_age = p('max_range_age_s')
        self.reset_after = p('reset_after_s')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.anchors = {}            # id -> (x, y, z) in base_link
        # Latest readings: id -> [range_m, stamp_s, used]. Measured ranges are in metres
        # without any offset.
        self.ranges = {}
        self.truth = {}
        self.calib_sums = {i: [0.0, 0] for i in self.anchor_ids}
        self.calib_start = None
        self.calibrated = not self.calibrate

        self.x = None                # EKF state [x, y, vx, vy] in odom
        self.P = None
        self.last_predict = None
        self.last_accept = None

        for anchor in self.anchor_ids:
            self.create_subscription(
                Float64, f"{p('range_topic')}/a{anchor}t{tag}",
                lambda msg, a=anchor: self.on_range(self.ranges, a, msg), 10)
            self.create_subscription(
                Float64, f"{p('truth_topic')}/a{anchor}t{tag}",
                lambda msg, a=anchor: self.on_range(self.truth, a, msg), 10)

        self.pub_person = self.create_publisher(PointStamped, '/uwb/person', 10)
        self.pub_person_odom = self.create_publisher(PointStamped, '/uwb/person_odom', 10)
        self.pub_velocity = self.create_publisher(Vector3Stamped, '/uwb/person_velocity', 10)
        self.pub_truth = self.create_publisher(PointStamped, '/uwb/person_truth', 10)
        self.pub_marker = self.create_publisher(Marker, '/uwb/person_marker', 10)
        self.create_timer(1.0 / p('rate_hz'), self.step)
        mode = (f'calibrating against ground truth for {self.calibration_s:g} s'
                if self.calibrate else f'range offsets {self.offsets}')
        self.get_logger().info(
            f'Anchors {self.anchor_ids}, tag {tag} at {self.tag_height:g} m; {mode}')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_range(self, store, anchor, msg):
        store[anchor] = [msg.data / 100.0, self.now_s(), False]

    # --- set-up ------------------------------------------------------------------

    def lookup_anchors(self):
        for anchor in self.anchor_ids:
            if anchor in self.anchors:
                continue
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, f'uwb_anchor_{anchor}', Time()).transform.translation
            except tf2_ros.TransformException:
                continue
            self.anchors[anchor] = (t.x, t.y, t.z)
            self.get_logger().info(
                f'Anchor {anchor} at ({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) in {self.base_frame}')
        return len(self.anchors) == len(self.anchor_ids)

    def run_calibration(self, now):
        """Average (true - measured) per anchor; True once done."""
        fresh = [a for a in self.anchor_ids
                 if a in self.ranges and a in self.truth and not self.ranges[a][2]]
        if self.calib_start is None:
            if len(fresh) < len(self.anchor_ids):
                return False
            self.calib_start = now
        for a in fresh:
            self.calib_sums[a][0] += self.truth[a][0] - self.ranges[a][0]
            self.calib_sums[a][1] += 1
            self.ranges[a][2] = True
        if now - self.calib_start < self.calibration_s:
            return False
        self.offsets = {a: s / max(n, 1) for a, (s, n) in self.calib_sums.items()}
        text = ', '.join(f'a{a}: {o * 100:+.1f} cm ({self.calib_sums[a][1]} ranges)'
                         for a, o in self.offsets.items())
        self.get_logger().info(f'Calibrated range offsets: {text}')
        return True

    # --- geometry ----------------------------------------------------------------

    def odom_from_base(self):
        """(x, y, yaw) of base_link in odom, or None."""
        try:
            t = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time()).transform
        except tf2_ros.TransformException:
            return None
        return t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation)

    @staticmethod
    def to_odom(pose, x, y):
        ox, oy, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return ox + c * x - s * y, oy + s * x + c * y

    @staticmethod
    def to_base(pose, x, y):
        ox, oy, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = x - ox, y - oy
        return c * dx + s * dy, -s * dx + c * dy

    def horizontal(self, anchor, slant):
        dz = self.tag_height - self.anchors[anchor][2]
        return math.sqrt(max(slant * slant - dz * dz, 1e-4))

    def least_squares_fix(self, store, offsets, now):
        """Person in base_link from all fresh ranges in store, or None."""
        use = [a for a in self.anchor_ids if a in store and now - store[a][1] <= self.max_age]
        if len(use) < 3:
            return None
        anchors_xy = [self.anchors[a][:2] for a in use]
        ranges = [self.horizontal(a, store[a][0] + offsets.get(a, 0.0)) for a in use]
        # The anchors are close together, so the linear start can land on the wrong side
        # when the ranges are noisy; also try a start in front of and behind the robot.
        best, best_cost = None, None
        for guess in (None, (1.5, 0.0), (-1.5, 0.0), (0.0, 1.5), (0.0, -1.5)):
            fix = solve_position(anchors_xy, ranges, guess)
            if fix is None or not np.all(np.isfinite(fix)):
                continue
            cost = sum((np.linalg.norm(fix - np.array(axy)) - r) ** 2
                       for axy, r in zip(anchors_xy, ranges))
            if best_cost is None or cost < best_cost:
                best, best_cost = fix, cost
        if best is None:
            return None
        # With the anchors this close together, noisy ranges can fit a far-off point;
        # reject fixes that are implausibly far or don't match the ranges.
        if (np.linalg.norm(best) > self.max_fix_distance
                or math.sqrt(best_cost / len(use)) > self.max_fix_rms):
            return None
        return best

    # --- filter ------------------------------------------------------------------

    def predict(self, now):
        dt = min(max(now - self.last_predict, 0.0), 0.5)
        self.last_predict = now
        if dt == 0.0:
            return
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        q1 = np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt ** 2]]) * self.accel_var
        q = np.zeros((4, 4))
        q[np.ix_([0, 2], [0, 2])] = q1
        q[np.ix_([1, 3], [1, 3])] = q1
        self.x = f @ self.x
        self.P = f @ self.P @ f.T + q

    def update_range(self, anchor_odom, anchor_z, measured):
        dx, dy = self.x[0] - anchor_odom[0], self.x[1] - anchor_odom[1]
        dz = self.tag_height - anchor_z
        predicted = math.sqrt(dx * dx + dy * dy + dz * dz)
        h = np.array([[dx / predicted, dy / predicted, 0.0, 0.0]])
        s = float(h @ self.P @ h.T) + self.range_var
        innovation = measured - predicted
        if innovation * innovation > self.gate * self.gate * s:
            return False
        k = (self.P @ h.T) / s
        self.x = self.x + k[:, 0] * innovation
        # Joseph form: keeps P symmetric and positive definite. The four range directions
        # are nearly parallel (anchors 17-24 cm apart), and the short form lost that and
        # let the estimate run away.
        i_kh = np.eye(4) - k @ h
        self.P = i_kh @ self.P @ i_kh.T + (k @ k.T) * self.range_var
        self.P = 0.5 * (self.P + self.P.T)
        speed = math.hypot(self.x[2], self.x[3])
        if speed > self.max_speed:
            self.x[2:] *= self.max_speed / speed
        return True

    def start_filter(self, fix_odom, now):
        self.x = np.array([fix_odom[0], fix_odom[1], 0.0, 0.0])
        self.P = np.diag([0.5, 0.5, 1.0, 1.0])
        self.last_predict = now
        self.last_accept = now

    # --- main loop ---------------------------------------------------------------

    def step(self):
        now = self.now_s()
        if not self.lookup_anchors():
            return
        if not self.calibrated:
            self.calibrated = self.run_calibration(now)
            return
        pose = self.odom_from_base()
        if pose is None:
            return

        truth = self.least_squares_fix(self.truth, {}, now)
        if truth is not None:
            self.publish_point(self.pub_truth, self.base_frame, truth)

        if self.x is None or now - self.last_accept > self.reset_after:
            fix = self.least_squares_fix(self.ranges, self.offsets, now)
            if fix is None:
                return
            if self.x is not None:
                self.get_logger().warn('No range accepted for a while; restarting the track')
            self.start_filter(self.to_odom(pose, *fix), now)
            for a in self.ranges:
                self.ranges[a][2] = True
        else:
            self.predict(now)
            for a in self.anchor_ids:
                reading = self.ranges.get(a)
                if reading is None or reading[2]:
                    continue
                reading[2] = True
                ax, ay, az = self.anchors[a]
                if self.update_range(self.to_odom(pose, ax, ay), az, reading[0] + self.offsets[a]):
                    self.last_accept = now

        person_base = self.to_base(pose, self.x[0], self.x[1])
        self.publish_point(self.pub_person, self.base_frame, person_base)
        self.publish_point(self.pub_person_odom, self.odom_frame, self.x[:2])
        velocity = Vector3Stamped()
        velocity.header.stamp = self.get_clock().now().to_msg()
        velocity.header.frame_id = self.odom_frame
        velocity.vector.x, velocity.vector.y = float(self.x[2]), float(self.x[3])
        self.pub_velocity.publish(velocity)
        self.publish_marker()

    def publish_point(self, publisher, frame, xy):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.point.x, msg.point.y = float(xy[0]), float(xy[1])
        msg.point.z = self.tag_height
        publisher.publish(msg)

    def publish_marker(self):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.odom_frame
        m.ns, m.id, m.type, m.action = 'uwb_person', 0, Marker.CYLINDER, Marker.ADD
        m.pose.position.x, m.pose.position.y = float(self.x[0]), float(self.x[1])
        m.pose.position.z = 0.9
        m.pose.orientation.w = 1.0
        # Width: two standard deviations of the position estimate, at least the body width.
        sigma = math.sqrt(max(self.P[0, 0], self.P[1, 1]))
        m.scale.x = m.scale.y = max(0.5, 4.0 * sigma)
        m.scale.z = 1.8
        m.color.r, m.color.g, m.color.b, m.color.a = 0.96, 0.55, 0.13, 0.5
        self.pub_marker.publish(m)


def main():
    rclpy.init()
    node = UwbLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
