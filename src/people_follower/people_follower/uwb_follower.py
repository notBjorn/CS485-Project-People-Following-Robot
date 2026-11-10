"""Follow the person tracked by uwb_localizer, along the path the person walked.

Driving straight at the person cuts corners: when the person walks through a door
and turns, the straight line runs into the wall. So this node records the person's
positions in odom as a trail of breadcrumbs (from /uwb/person_odom) and drives along
the trail, staying follow_distance behind the person (measured along the trail).

    - Steering: aim at the first breadcrumb at least lookahead_m from the robot.
      If it is more than turn_in_place_rad off the robot's heading, turn on the spot.
    - Speed: the person's own pace (their estimated velocity towards where the
      robot is heading, from /uwb/person_velocity) plus a catch-up term
      proportional to how far the robot is behind its spot on the trail, up to
      max_linear, with an acceleration limit. So it closes a gap faster than the
      person walks, then settles to their pace.
    - Slowing down, by distance to the person along the trail (defaults):
          beyond 1.5 m   up to max_linear
          1.5 -> 1.0 m   braking: the cap falls from max_linear to creep_speed
          1.0 -> 0.5 m   slow zone: the cap falls from creep_speed to 0
          0.5 m          stop (follow_distance)
      While the person walks, the robot keeps pace where the cap equals their
      speed (about 1.3 m behind at 0.2 m/s); when they stop, it creeps in to 0.5 m.
    - Safety: no forward motion while the LiDAR sees something closer than
      obstacle_stop_m within obstacle_half_angle_rad of straight ahead (turning
      is still allowed), and a stop if the person estimate is older than lost_after_s.

Subscribes /uwb/person_odom (geometry_msgs/PointStamped), /uwb/person_velocity
(geometry_msgs/Vector3Stamped) and /scan; publishes
/cmd_vel (geometry_msgs/TwistStamped) and the trail as /uwb/trail (nav_msgs/Path).
The robot must be undocked first (scripts/launch_follow.sh does it).
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


class UwbFollower(Node):

    def __init__(self):
        super().__init__('uwb_follower')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        # m behind the person (to the tag on their back), along the trail
        self.declare_parameter('follow_distance', 0.5)
        self.declare_parameter('slow_zone_m', 0.5)           # creep over this last stretch
        self.declare_parameter('braking_m', 0.5)             # and brake over this before it
        self.declare_parameter('creep_speed', 0.08)          # m/s at the slow zone's outer edge
        self.declare_parameter('breadcrumb_spacing', 0.25)   # m between trail points
        # Aim this far along the trail: further smooths out the wobble of the UWB estimate
        self.declare_parameter('lookahead_m', 1.0)
        self.declare_parameter('reached_m', 0.3)             # trail points this close are done
        self.declare_parameter('max_linear', 0.3)            # m/s (TurtleBot 4 top speed ~0.31)
        self.declare_parameter('max_angular', 1.2)           # rad/s
        self.declare_parameter('max_accel', 0.5)             # m/s^2
        self.declare_parameter('k_linear', 0.5)              # catch-up: m/s per m behind
        self.declare_parameter('k_angular', 1.5)
        # Turn on the spot only beyond this; below it, drive and turn (slower the further off)
        self.declare_parameter('turn_in_place_rad', 1.2)
        self.declare_parameter('obstacle_stop_m', 0.35)      # from the LiDAR, which is ~centred
        self.declare_parameter('obstacle_half_angle_rad', 0.5)
        self.declare_parameter('lost_after_s', 0.5)
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
        self.obstacle_angle = p('obstacle_half_angle_rad')
        self.lost_after = p('lost_after_s')
        self.period = 1.0 / p('rate_hz')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.person = None           # (x, y) in odom
        self.person_time = None
        self.person_velocity = (0.0, 0.0)   # odom, m/s
        self.trail = []              # breadcrumbs in odom, oldest first
        self.front_clearance = math.inf
        self.scan_yaw = None         # yaw of the LiDAR frame in base_link
        self.linear = 0.0
        self.was_driving = None

        self.create_subscription(PointStamped, '/uwb/person_odom', self.on_person, 10)
        self.create_subscription(Vector3Stamped, '/uwb/person_velocity', self.on_velocity, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.trail_pub = self.create_publisher(Path, '/uwb/trail', 10)
        self.create_timer(self.period, self.step)
        self.get_logger().info(
            f'Following {self.follow_distance:g} m behind the person, slowing over the last '
            f'{self.slow_zone:g} m, up to {self.max_linear:g} m/s')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_person(self, msg):
        self.person = (msg.point.x, msg.point.y)
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
            if not (msg.range_min < r < msg.range_max):
                continue
            angle = wrap_angle(msg.angle_min + i * msg.angle_increment + self.scan_yaw)
            if abs(angle) <= self.obstacle_angle:
                closest = min(closest, r)
        self.front_clearance = closest

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

    def log_state(self, driving, why):
        if driving != self.was_driving:
            self.was_driving = driving
            self.get_logger().info(why)

    def step(self):
        pose = self.robot_pose()
        if pose is None or self.person is None:
            self.publish(0.0, 0.0)
            return
        if self.now_s() - self.person_time > self.lost_after:
            self.log_state(False, 'Person estimate lost; stopped')
            self.publish(0.0, 0.0)
            return
        rx, ry, ryaw = pose

        # Drop breadcrumbs the robot has reached (and everything before them).
        for i in range(len(self.trail) - 1, -1, -1):
            if math.dist(self.trail[i], (rx, ry)) < self.reached:
                del self.trail[:i + 1]
                break

        # Distance to the person along the trail (robot -> breadcrumbs -> person).
        points = [(rx, ry)] + self.trail + [self.person]
        along = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        self.publish_trail()

        if along <= self.follow_distance:
            # Close enough: stop, but keep facing the person.
            bearing = wrap_angle(math.atan2(self.person[1] - ry, self.person[0] - rx) - ryaw)
            angular = clamp(self.k_angular * bearing, -self.max_angular, self.max_angular)
            # Close up the estimate's direction wobbles by 10-20 degrees; don't chase it.
            if abs(bearing) < 0.35:
                angular = 0.0
            self.log_state(False, f'Within {self.follow_distance:g} m of the person; waiting')
            self.publish(0.0, angular)
            return

        target = self.person
        for point in self.trail:
            if math.dist(point, (rx, ry)) >= self.lookahead:
                target = point
                break
        heading_error = wrap_angle(math.atan2(target[1] - ry, target[0] - rx) - ryaw)
        angular = clamp(self.k_angular * heading_error, -self.max_angular, self.max_angular)

        if abs(heading_error) > self.turn_in_place:
            self.log_state(False, 'Turning towards the trail')
            self.publish(0.0, angular)
            return
        if self.front_clearance < self.obstacle_stop:
            self.log_state(False, f'Obstacle {self.front_clearance:.2f} m ahead; waiting')
            self.publish(0.0, angular)
            return

        gap = along - self.follow_distance
        # The person's pace in the direction the robot is heading for: negative when they
        # walk back towards it.
        aim = math.atan2(target[1] - ry, target[0] - rx)
        person_pace = (self.person_velocity[0] * math.cos(aim)
                       + self.person_velocity[1] * math.sin(aim))
        linear = clamp(person_pace + self.k_linear * gap, 0.0, self.max_linear)
        linear = min(linear, self.speed_cap(gap))
        linear *= max(0.0, math.cos(heading_error))
        self.log_state(True, 'Following')
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
    node = UwbFollower()
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
