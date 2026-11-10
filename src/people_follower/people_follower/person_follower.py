"""Follow the person picked by person_tracker (camera + YOLO), along the path they walked.

Adapted from uwb_follower on the map-and-UWB-plugin branch; the trail following and
speed control are the same, the input is the camera's target, and it searches when
the person isn't in view (the camera only sees 72 degrees ahead).

Driving straight at the person cuts corners: when the person walks through a door
and turns, the straight line runs into the wall. So this node records the target's
positions in odom as a trail of breadcrumbs and drives along the trail, staying
follow_distance behind the person (measured along the trail, to their body centre).

    - Steering: aim at the first breadcrumb at least lookahead_m from the robot.
      If it is more than turn_in_place_rad off the robot's heading, turn on the spot.
    - Speed: the person's own pace (their velocity towards where the robot is
      heading) plus a catch-up term proportional to how far the robot is behind its
      spot on the trail, up to max_linear, with an acceleration limit.
    - Slowing down, by distance to the person along the trail (defaults):
          beyond 2.0 m   up to max_linear
          2.0 -> 1.5 m   braking: the cap falls from max_linear to creep_speed
          1.5 -> 1.0 m   slow zone: the cap falls from creep_speed to 0
          1.0 m          stop (follow_distance)
      Stopped, it turns to keep facing the person, which keeps them in view.
    - Trail upkeep: points before the one nearest the robot are dropped (the robot is
      past them). When the person reappears more than trail_jump_m from the end of
      the trail (they were out of view), the trail restarts from there instead of
      joining the two with a straight line, which could run through a wall or box.
    - Lost (no target for lost_after_s; person_tracker keeps a target for 2 s after
      its last detection): drive on along the trail to where the person was last seen,
      then turn on the spot (search_turn_rate) towards the side they were last seen
      on, until the camera finds someone. If an obstacle blocks the way there for
      blocked_search_s, it starts turning to look straight away. At the start,
      before anyone is seen, it turns on the spot too.
    - Safety: no forward motion while the LiDAR sees something closer than
      obstacle_stop_m within obstacle_half_angle_rad of straight ahead, until it is
      clear by obstacle_resume_m (turning is still allowed). Returns too close for the
      LiDAR to measure (at or below its minimum range) count as obstacles, not as
      nothing. And it never drives closer than follow_distance in a straight line,
      whatever the trail length.

Subscribes /person/target_odom (geometry_msgs/PointStamped), /person/target_velocity
(geometry_msgs/Vector3Stamped) and /scan; publishes /cmd_vel (geometry_msgs/TwistStamped)
and the trail as /person/trail (nav_msgs/Path). The robot must be undocked first
(scripts/launch_camera_follow.sh does it).
"""

import math

from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(value, low, high):
    return max(low, min(high, value))


class PersonFollower(Node):

    def __init__(self):
        super().__init__('person_follower')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        # m behind the person (to their body centre), along the trail. The camera has no
        # depth closer than 0.35 m and the robot's front is 0.17 m from its centre.
        self.declare_parameter('follow_distance', 1.0)
        self.declare_parameter('slow_zone_m', 0.5)           # creep over this last stretch
        self.declare_parameter('braking_m', 0.5)             # and brake over this before it
        self.declare_parameter('creep_speed', 0.08)          # m/s at the slow zone's outer edge
        self.declare_parameter('breadcrumb_spacing', 0.25)   # m between trail points
        # Aim this far along the trail: further smooths out the wobble of the estimate
        self.declare_parameter('lookahead_m', 1.0)
        self.declare_parameter('reached_m', 0.3)             # trail points this close are done
        self.declare_parameter('max_linear', 0.3)            # m/s (TurtleBot 4 top speed ~0.31)
        self.declare_parameter('max_angular', 1.2)           # rad/s
        self.declare_parameter('max_accel', 0.5)             # m/s^2
        self.declare_parameter('k_linear', 0.5)              # catch-up: m/s per m behind
        self.declare_parameter('k_angular', 1.5)
        # Turn on the spot only beyond this; below it, drive and turn (slower the further off)
        self.declare_parameter('turn_in_place_rad', 1.2)
        # m from the LiDAR, which is ~4 cm behind the robot's centre; the robot's edge is
        # 0.17 m from its centre, so this leaves about a third of a metre
        self.declare_parameter('obstacle_stop_m', 0.5)
        self.declare_parameter('obstacle_resume_m', 0.6)     # clear by this to drive on
        self.declare_parameter('obstacle_half_angle_rad', 0.7)
        self.declare_parameter('lost_after_s', 0.5)
        self.declare_parameter('search_turn_rate', 0.5)      # rad/s while looking for the person
        self.declare_parameter('trail_jump_m', 1.0)          # restart the trail on a jump this big
        self.declare_parameter('blocked_search_s', 3.0)      # lost and blocked this long: search
        self.declare_parameter('target_topic', '/person/target_odom')
        self.declare_parameter('velocity_topic', '/person/target_velocity')
        # camera_follow.launch.py sends this through safety_guard (/cmd_vel_unsafe)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('rate_hz', 20.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.base_frame = p('base_frame')
        self.odom_frame = p('odom_frame')
        self.follow_distance = p('follow_distance')
        self.slow_zone = p('slow_zone_m')
        self.braking = p('braking_m')
        self.creep_speed = p('creep_speed')
        self.spacing = p('breadcrumb_spacing')
        self.lookahead = p('lookahead_m')
        self.reached = p('reached_m')
        self.max_linear = p('max_linear')
        self.max_angular = p('max_angular')
        self.max_accel = p('max_accel')
        self.k_linear = p('k_linear')
        self.k_angular = p('k_angular')
        self.turn_in_place = p('turn_in_place_rad')
        self.obstacle_stop = p('obstacle_stop_m')
        self.obstacle_resume = p('obstacle_resume_m')
        self.obstacle_angle = p('obstacle_half_angle_rad')
        self.lost_after = p('lost_after_s')
        self.search_rate = p('search_turn_rate')
        self.trail_jump = p('trail_jump_m')
        self.blocked_search = p('blocked_search_s')
        self.period = 1.0 / p('rate_hz')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.person = None           # (x, y) in odom
        self.person_time = None
        self.person_velocity = (0.0, 0.0)   # odom, m/s
        self.trail = []              # breadcrumbs in odom, oldest first
        self.front_clearance = math.inf
        self.blocked = False         # obstacle stop in force (with hysteresis)
        self.scan_yaw = None         # yaw of the LiDAR frame in base_link
        self.linear = 0.0
        self.was_driving = None
        self.search_sign = 1.0       # which way to turn when searching (+ = left)
        self.blocked_since = None    # when an obstacle started blocking the lost-mode drive

        self.create_subscription(PointStamped, p('target_topic'), self.on_person, 10)
        self.create_subscription(Vector3Stamped, p('velocity_topic'), self.on_velocity, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(TwistStamped, p('cmd_topic'), 10)
        self.trail_pub = self.create_publisher(Path, '/person/trail', 10)
        self.create_timer(self.period, self.step)
        self.get_logger().info(
            f'Following {self.follow_distance:g} m behind the person, slowing over the last '
            f'{self.slow_zone:g} m, up to {self.max_linear:g} m/s')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_person(self, msg):
        person = (msg.point.x, msg.point.y)
        last = self.trail[-1] if self.trail else self.person
        if last is not None and math.dist(last, person) > self.trail_jump:
            # Seen again somewhere else: don't join the two with a straight line.
            self.trail = []
            self.get_logger().info('Person reappeared away from the trail; starting a new trail')
        self.person = person
        self.person_time = self.now_s()
        if not self.trail or math.dist(self.trail[-1], self.person) >= self.spacing:
            self.trail.append(self.person)

    def on_velocity(self, msg):
        self.person_velocity = (msg.vector.x, msg.vector.y)

    def on_scan(self, msg):
        if self.scan_yaw is None:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, msg.header.frame_id, Time()).transform
            except tf2_ros.TransformException:
                return
            self.scan_yaw = yaw_from_quaternion(t.rotation)
        closest = math.inf
        for i, r in enumerate(msg.ranges):
            # Beyond range (inf) or invalid (nan): nothing there. At or below the minimum
            # range: something too close to measure (Gazebo reports range_min), so count
            # it as touching rather than ignoring it.
            if not math.isfinite(r) or r >= msg.range_max:
                continue
            angle = wrap_angle(msg.angle_min + i * msg.angle_increment + self.scan_yaw)
            if abs(angle) <= self.obstacle_angle:
                closest = min(closest, 0.0 if r <= msg.range_min else r)
        self.front_clearance = closest
        if closest < self.obstacle_stop:
            self.blocked = True
        elif closest > self.obstacle_resume:
            self.blocked = False

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time()).transform
        except tf2_ros.TransformException:
            return None
        return t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation)

    def publish(self, linear, angular):
        # Acceleration limit on the way up; stopping is immediate.
        if linear > self.linear:
            linear = min(linear, self.linear + self.max_accel * self.period)
        self.linear = linear
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)

    def log_state(self, state, why):
        if state != self.was_driving:
            self.was_driving = state
            self.get_logger().info(why)

    def step(self):
        pose = self.robot_pose()
        if pose is None:
            self.publish(0.0, 0.0)
            return
        rx, ry, ryaw = pose
        if self.person is None:
            self.log_state('search', 'No one seen yet; turning to look')
            self.publish(0.0, self.search_sign * self.search_rate)
            return
        lost = self.now_s() - self.person_time > self.lost_after
        if not lost:
            self.blocked_since = None
            # Remember which side the person is on, for searching if they're lost.
            side = wrap_angle(math.atan2(self.person[1] - ry, self.person[0] - rx) - ryaw)
            self.search_sign = 1.0 if side >= 0.0 else -1.0

        # Drop the breadcrumbs the robot is past: everything before the one nearest it,
        # and that one too once it is within reached_m.
        if self.trail:
            nearest = min(range(len(self.trail)),
                          key=lambda i: math.dist(self.trail[i], (rx, ry)))
            del self.trail[:nearest]
            if math.dist(self.trail[0], (rx, ry)) < self.reached:
                del self.trail[0]

        # Distance to the person along the trail (robot -> breadcrumbs -> person), and in a
        # straight line. Speed and stopping use the shorter: the trail can be longer than
        # the real gap (e.g. the person walked back towards the robot).
        points = [(rx, ry)] + self.trail + [self.person]
        along = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        direct = math.dist((rx, ry), self.person)
        along = min(along, direct) if not lost else along
        self.publish_trail()

        if lost:
            # Go to where the person was last seen, then turn on the spot to look.
            blocked = self.blocked
            if blocked and self.blocked_since is None:
                self.blocked_since = self.now_s()
            elif not blocked:
                self.blocked_since = None
            stuck = (self.blocked_since is not None
                     and self.now_s() - self.blocked_since > self.blocked_search)
            if along <= self.reached or stuck:
                self.log_state('search', 'Person lost; turning to look for them')
                self.publish(0.0, self.search_sign * self.search_rate)
                return
            self.person_velocity = (0.0, 0.0)
            self.drive_along_trail(pose, along, 0.0, 'Person lost; going to where they were seen')
            return

        if along <= self.follow_distance:
            # Close enough: stop, but keep facing the person.
            bearing = wrap_angle(math.atan2(self.person[1] - ry, self.person[0] - rx) - ryaw)
            angular = clamp(self.k_angular * bearing, -self.max_angular, self.max_angular)
            # Close up the estimate's direction wobbles by 10-20 degrees; don't chase it.
            if abs(bearing) < 0.35:
                angular = 0.0
            self.log_state('wait', f'Within {self.follow_distance:g} m of the person; waiting')
            self.publish(0.0, angular)
            return
        self.drive_along_trail(pose, along, self.follow_distance, 'Following')

    def drive_along_trail(self, pose, along, stop_at, why):
        """Drive along the trail, stopping stop_at metres (along it) short of its end."""
        rx, ry, ryaw = pose
        target = self.person
        for point in self.trail:
            if math.dist(point, (rx, ry)) >= self.lookahead:
                target = point
                break
        heading_error = wrap_angle(math.atan2(target[1] - ry, target[0] - rx) - ryaw)
        angular = clamp(self.k_angular * heading_error, -self.max_angular, self.max_angular)

        if abs(heading_error) > self.turn_in_place:
            self.log_state('turn', 'Turning towards the trail')
            self.publish(0.0, angular)
            return
        if self.blocked:
            self.log_state('obstacle', f'Obstacle {self.front_clearance:.2f} m ahead; waiting')
            self.publish(0.0, angular)
            return

        gap = along - stop_at
        # The person's pace in the direction the robot is heading for: negative when they
        # walk back towards it.
        aim = math.atan2(target[1] - ry, target[0] - rx)
        person_pace = (self.person_velocity[0] * math.cos(aim)
                       + self.person_velocity[1] * math.sin(aim))
        linear = clamp(person_pace + self.k_linear * gap, 0.0, self.max_linear)
        linear = min(linear, self.speed_cap(gap))
        linear *= max(0.0, math.cos(heading_error))
        self.log_state(why, why)
        self.publish(linear, angular)

    def speed_cap(self, gap):
        """Top speed with the robot gap metres short of its stopping spot."""
        if gap < self.slow_zone:
            return self.creep_speed * gap / self.slow_zone
        if gap < self.slow_zone + self.braking:
            fraction = (gap - self.slow_zone) / self.braking
            return self.creep_speed + fraction * (self.max_linear - self.creep_speed)
        return self.max_linear

    def publish_trail(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        for x, y in self.trail:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.trail_pub.publish(msg)

    def stop(self):
        self.linear = 0.0
        self.publish(0.0, 0.0)


def main():
    rclpy.init()
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
