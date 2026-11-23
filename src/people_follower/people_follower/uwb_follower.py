"""Follow the person tracked by uwb_localizer, along the path the person walked.

Driving straight at the person cuts corners: when the person walks through a door
and turns, the straight line runs into the wall. So this node records the person's
positions in odom as a trail of breadcrumbs (from /uwb/person_odom) and drives along
the trail, staying follow_distance behind the person (measured along the trail).

    - Breadcrumbs the robot has passed are dropped: all those before the nearest one in
      the next 3 m of trail, so a corner it cut doesn't send it back.
    - Steering: aim at the first breadcrumb at least lookahead_m from the robot, if the
      straight line to it is clear for the robot's width (the LiDAR's last scan);
      otherwise at the furthest breadcrumb before it that is. So at a corner or a
      doorway the robot keeps to the person's path instead of cutting across it into
      the door frame. If the target is more than turn_in_place_rad off the robot's
      heading, turn on the spot.
    - Speed: the person's own pace (their estimated velocity towards where the
      robot is heading, from /uwb/person_velocity) plus a catch-up term
      proportional to how far the robot is behind its spot on the trail, up to
      max_linear, with an acceleration limit. So it closes a gap faster than the
      person walks, then settles to their pace.
    - Slowing down near the person, measured straight-line from the robot's edge
      (robot_radius_m from its centre) to the tag on the person's back:
          beyond 6 ft (person_slow_m, 1.83 m)  up to max_linear
          6 ft -> 2 ft                         the cap falls in proportion to 0
          2 ft (person_stop_m, 0.61 m)         no forward motion; it only turns
      follow_distance (along the trail, robot centre to tag) is raised to at least
      robot_radius_m + person_stop_m, so it also stops 2 ft away; over the last
      slow_zone_m + braking_m of the trail it eases in further, to creep_speed.
    - Objects (LiDAR) in the robot's path, a corridor its width plus
      obstacle_side_margin_m each side, measured from its front edge:
          beyond obstacle_slow_m (0.6 m, 2 ft)  up to max_linear
          0.6 m -> 0.2 m                        the cap falls to creep_speed
          obstacle_stop_m (0.2 m)               no forward motion; it only turns
      The LiDAR points are placed with TF, so this holds wherever the LiDAR is
      mounted; points inside the robot's own body are ignored.
    - Stepping aside: if the person walks towards the robot (at least yield_speed_mps,
      from the UWB velocity) and the robot is in their way (within yield_ahead_m ahead
      of them and yield_lateral_m of the line they're walking) for yield_confirm_s (so
      one noisy velocity estimate doesn't set it off), the robot turns on the spot
      towards whichever side of that line the LiDAR shows is clear (preferring the side
      back towards where the robot came from, where the person has already been; else
      its own side) and drives forwards, so it can see where it goes, until it is
      yield_lateral_m to the side. If neither side has room (a corridor), it drives on
      ahead of them, faster than they walk, until one does. It waits aside until they've
      walked past (or are 2.5 m away, or yield_timeout_s), planning again if they turn
      towards it again (at most every 1.5 s, keeping its side), then follows again. With
      no room at all, or once they're within person_backup_m, it backs away (below).
    - Backing away: the robot reverses (up to backup_speed, turning to face the
      person) when the person comes within person_backup_m (1.5 ft) of its edge, or
      when the LiDAR sees something closer than obstacle_stop_m ahead while the
      person is within person_stop_m + resume_margin_m (2.5 ft: so it is most likely
      them, not a wall). It stops reversing once it is person_stop_m (2 ft) from them
      and clear ahead, then waits until they are resume_margin_m further away before
      following again, so it doesn't edge back in. It won't reverse into anything the
      LiDAR sees within obstacle_stop_m behind it, slowing as it gets near. If the
      LiDAR can't see straight back, it reverses at most blind_backup_m before it next
      drives forward.
    - Curves: while the robot turns as it drives, the LiDAR is also checked along the
      arc it is driving (from its last command), so a wall coming in from the side
      counts as soon as the robot is heading into it.
    - Getting unstuck: if something blocks the path for stuck_after_s, the robot first
      backs up stuck_backup_m (with the same checks behind it as backing away from the
      person), then skips the trail point it was heading for, and those within
      lookahead_m of it, and aims at the next one.
      After stuck_attempts tries without getting stuck_progress_m from where it got
      stuck, it stops and waits ('stuck').
    - Braking distance: every speed limit above (person, objects, end of the trail) is
      also kept at or below sqrt(2 * max_decel * distance left), so the robot can
      always stop in time with its base's braking limit (TIAGo's controller brakes
      at 0.5 m/s^2: 1 m to stop from its top speed of 1 m/s).
    - LiDAR clearances are reduced by how far the robot will have driven since the
      scan by the time a command takes effect (scan age + one cycle + latency_s), so
      the limits hold between scans and with delays.
    - Unreliable estimate: while /uwb/person_uncertainty (from uwb_localizer) is above
      max_uncertainty_m, until it drops below reliable_uncertainty_m, the estimates are
      ignored: no breadcrumbs from them and nothing chased, so after lost_after_s it is
      the same as a lost signal (below), going to the last reliable position. UWB
      through walls is noisy and biased enough to put the person metres off; the
      localizer's uncertainty shows it. Without that topic every estimate is trusted.
    - Signal lost (no person estimate for lost_after_s): the robot carries on along the
      trail, at up to lost_speed_mps, to the last place it knew the person was, and
      stops person_stop_m (2 ft) short of it, in case they're standing there out of
      signal; then it waits there. All the obstacle checks still apply; it doesn't step
      aside or back away from a person it can't see. When the estimate comes back it
      follows as usual.

Subscribes /uwb/person_odom (geometry_msgs/PointStamped), /uwb/person_velocity
(geometry_msgs/Vector3Stamped), /uwb/person_uncertainty (std_msgs/Float64) and /scan;
publishes
/cmd_vel (geometry_msgs/TwistStamped) and the trail as /uwb/trail (nav_msgs/Path).
The TurtleBot 4 must be undocked first (scripts/launch_follow.sh does it; TIAGo has no dock).

Starting and stopping (the trail keeps recording either way):
    start_enabled (parameter, default true): follow from the start, as before.
    /uwb_follower/enable  std_srvs/SetBool: true follows, false stops the robot and
        sends no more drive commands (so teleop can take over).
    /uwb_follower/follow_person  people_follower_interfaces/action/FollowPerson: follow
        until cancelled or a limit in the goal is reached, with feedback on what the
        robot is doing; the robot stops when the goal ends. A new goal replaces the
        current one, and enable false aborts it.
"""

import math

import numpy as np

from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Path
from people_follower_interfaces.action import FollowPerson
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
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
        # m behind the person (centre to the tag on their back), along the trail; at least
        # robot_radius_m + person_stop_m
        self.declare_parameter('follow_distance', 0.79)
        self.declare_parameter('robot_radius_m', 0.18)       # TurtleBot 4; TIAGo 0.27
        self.declare_parameter('person_slow_m', 1.83)        # 6 ft from the robot's edge
        self.declare_parameter('person_stop_m', 0.61)        # 2 ft from the robot's edge
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
        self.declare_parameter('max_decel', 0.5)             # m/s^2 the base can brake at
        self.declare_parameter('k_linear', 0.5)              # catch-up: m/s per m behind
        self.declare_parameter('k_angular', 1.5)
        # Turn on the spot only beyond this; below it, drive and turn (slower the further off)
        self.declare_parameter('turn_in_place_rad', 1.2)
        self.declare_parameter('obstacle_stop_m', 0.2)       # from the robot's front edge
        self.declare_parameter('obstacle_slow_m', 0.6)       # 2 ft: slow for objects within
        self.declare_parameter('person_backup_m', 0.45)      # 1.5 ft: back away from the person
        self.declare_parameter('backup_speed', 0.15)         # m/s, top reversing speed
        self.declare_parameter('blind_backup_m', 0.5)        # reversing limit with no rear view
        self.declare_parameter('resume_margin_m', 0.15)      # after backing: wait for this more
        self.declare_parameter('latency_s', 0.1)             # command-to-wheels delay allowance
        self.declare_parameter('obstacle_side_margin_m', 0.05)  # path corridor: body + this
        self.declare_parameter('yield_speed_mps', 0.15)      # person walking at least this fast
        self.declare_parameter('yield_ahead_m', 3.0)         # ... with the robot this far ahead
        self.declare_parameter('yield_lateral_m', 0.8)       # ... and this close to their line
        self.declare_parameter('yield_max_m', 1.5)           # most it drives to step aside
        self.declare_parameter('yield_timeout_s', 12.0)      # most it spends stepping aside
        self.declare_parameter('yield_confirm_s', 0.6)       # heading at it this long first
        self.declare_parameter('stuck_after_s', 2.0)         # blocked this long: get unstuck
        self.declare_parameter('stuck_backup_m', 0.3)        # by backing up this far first
        self.declare_parameter('stuck_attempts', 3)          # tries before giving up
        self.declare_parameter('stuck_progress_m', 1.0)      # forward driving that resets tries
        self.declare_parameter('lost_after_s', 0.5)
        self.declare_parameter('lost_speed_mps', 0.4)        # going to their last position
        self.declare_parameter('max_uncertainty_m', 0.4)     # estimates worse: ignored ...
        self.declare_parameter('reliable_uncertainty_m', 0.3)  # ... until better than this
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('start_enabled', True)   # false: wait for enable / an action

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.base_frame = p('base_frame')
        self.odom_frame = p('odom_frame')
        self.robot_radius = p('robot_radius_m')
        self.person_slow = p('person_slow_m')
        self.person_stop = p('person_stop_m')
        self.default_follow_distance = self.safe_follow_distance(p('follow_distance'))
        self.follow_distance = self.default_follow_distance
        self.slow_zone = p('slow_zone_m')
        self.braking = p('braking_m')
        self.creep_speed = p('creep_speed')
        self.spacing = p('breadcrumb_spacing')
        self.lookahead = p('lookahead_m')
        self.reached = p('reached_m')
        self.max_linear = p('max_linear')
        self.max_angular = p('max_angular')
        self.max_accel = p('max_accel')
        self.max_decel = p('max_decel')
        self.k_linear = p('k_linear')
        self.k_angular = p('k_angular')
        self.turn_in_place = p('turn_in_place_rad')
        self.obstacle_stop = p('obstacle_stop_m')
        self.obstacle_slow = p('obstacle_slow_m')
        self.person_backup = p('person_backup_m')
        self.backup_speed = p('backup_speed')
        self.blind_backup = p('blind_backup_m')
        self.resume_margin = p('resume_margin_m')
        self.latency = p('latency_s')
        self.corridor = self.robot_radius + p('obstacle_side_margin_m')
        self.yield_speed = p('yield_speed_mps')
        self.yield_ahead = p('yield_ahead_m')
        self.yield_lateral = p('yield_lateral_m')
        self.yield_max = p('yield_max_m')
        self.yield_timeout = p('yield_timeout_s')
        self.yield_confirm = p('yield_confirm_s')
        self.stuck_after = p('stuck_after_s')
        self.stuck_backup = p('stuck_backup_m')
        self.stuck_attempts = p('stuck_attempts')
        self.stuck_progress = p('stuck_progress_m')
        self.lost_after = p('lost_after_s')
        self.lost_speed = p('lost_speed_mps')
        self.max_uncertainty = p('max_uncertainty_m')
        self.reliable_uncertainty = p('reliable_uncertainty_m')
        self.reliable = True         # whether the localizer's estimates are trusted
        self.uncertainty_seen = False  # the localizer publishes /uwb/person_uncertainty
        self.pending = None          # (position, velocity) waiting for its uncertainty
        self.period = 1.0 / p('rate_hz')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.person = None           # (x, y) in odom
        self.person_time = None
        self.person_velocity = (0.0, 0.0)   # odom, m/s
        self.trail = []              # breadcrumbs in odom, oldest first
        self.front_clearance = math.inf
        self.front_arc = math.inf
        self.rear_clearance = math.inf
        self.scan_odom = np.empty((0, 2))   # last scan's points in odom
        self.rear_visible = False    # whether the LiDAR's field of view includes straight back
        self.backing = False
        self.backed = 0.0            # m reversed since the robot last drove forward
        self.holding = False         # backed away; waiting for the person to move off
        self.scan_time = None        # when the clearances were measured
        self.scan_pose = None        # (x, y, yaw) of the LiDAR frame in base_link
        self.linear = 0.0
        self.angular = 0.0
        self.logged_state = None
        self.blocked_since = None    # when the path ahead became blocked
        self.aside = None            # stepping aside: dict of direction, start, ... or None
        self.in_way_since = None     # when the person started heading at the robot
        self.prefer_side = None      # side to keep to when stepping aside is replanned
        self.lost = False            # no estimate: going to where the person was last seen
        self.history = []            # where the robot has been (odom), every 0.25 m, newest last
        self.unstick_from = None     # (x, y) where backing up to get unstuck started
        self.unstick_skip = 0        # trail points to drop once backed up
        self.unstick_tries = 0
        self.stuck_at = None         # (x, y) where the robot last got stuck
        self.target_index = None     # trail index the robot is aiming at (None: the person)
        # Everything runs on one thread (a single-threaded executor): rclpy's
        # multi-threaded executor stalled the 20 Hz control loop for seconds at a time.
        # A FollowPerson goal is a coroutine that waits while action_tick() runs it.
        self.enabled = p('start_enabled')
        self.state = 'no_estimate' if self.enabled else 'disabled'  # for feedback
        self.along = math.nan        # m to the person along the trail
        self.driven = 0.0            # m driven, for FollowPerson results
        self.last_xy = None
        # The active FollowPerson goal: handle, request, start, and the future execute()
        # waits on.
        self.goal = None

        self.create_subscription(PointStamped, '/uwb/person_odom', self.on_person, 10)
        self.create_subscription(Vector3Stamped, '/uwb/person_velocity', self.on_velocity, 10)
        self.create_subscription(Float64, '/uwb/person_uncertainty', self.on_uncertainty, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.trail_pub = self.create_publisher(Path, '/uwb/trail', 10)
        self.create_timer(self.period, self.step)
        self.create_service(SetBool, '~/enable', self.on_enable)
        self.action_server = ActionServer(
            self, FollowPerson, '~/follow_person', execute_callback=self.execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT)
        self.create_timer(0.2, self.action_tick)
        self.get_logger().info(
            f'Following {self.follow_distance:g} m behind the person, up to '
            f'{self.max_linear:g} m/s; slowing within {self.person_slow:g} m of them and '
            f'stopping at {self.person_stop:g} m (from the robot\'s edge)'
            + ('' if self.enabled else
               ' (waiting: call ~/enable or send a FollowPerson goal to start)'))

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_uncertainty(self, msg):
        self.uncertainty_seen = True
        if self.reliable and msg.data > self.max_uncertainty:
            self.reliable = False
            self.get_logger().info(
                f'UWB estimate unreliable (uncertainty {msg.data:.2f} m); ignoring it')
        elif not self.reliable and msg.data < self.reliable_uncertainty:
            self.reliable = True
            self.get_logger().info(
                f'UWB estimate reliable again (uncertainty {msg.data:.2f} m)')
        # Use the estimate this uncertainty goes with only if it is reliable: no
        # following, or breadcrumbs, from a bad one.
        if self.pending is not None and self.reliable:
            self.use_estimate(*self.pending)
        self.pending = None

    def on_person(self, msg):
        position = (msg.point.x, msg.point.y)
        if self.uncertainty_seen:
            # Hold it until its uncertainty arrives (the two are separate messages).
            velocity = self.pending[1] if self.pending else self.person_velocity
            self.pending = (position, velocity)
        else:
            self.use_estimate(position, self.person_velocity)

    def on_velocity(self, msg):
        velocity = (msg.vector.x, msg.vector.y)
        if self.uncertainty_seen:
            if self.pending is not None:
                self.pending = (self.pending[0], velocity)
        else:
            self.person_velocity = velocity

    def use_estimate(self, position, velocity):
        self.person_velocity = velocity
        self.person = position
        self.person_time = self.now_s()
        if not self.trail or math.dist(self.trail[-1], self.person) >= self.spacing:
            self.trail.append(self.person)

    def safe_follow_distance(self, distance):
        """follow_distance raised, if needed, so the robot stops person_stop_m away."""
        least = self.robot_radius + self.person_stop
        if distance < least:
            self.get_logger().warn(
                f'follow_distance {distance:g} m would end closer than {self.person_stop:g} m '
                f'to the person; using {least:g} m')
        return max(distance, least)

    def on_scan(self, msg):
        if self.scan_pose is None:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, msg.header.frame_id, Time()).transform
            except tf2_ros.TransformException:
                return
            self.scan_pose = (t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))
        sx, sy, syaw = self.scan_pose
        r = np.asarray(msg.ranges, dtype=float)
        angle = msg.angle_min + np.arange(len(r)) * msg.angle_increment + syaw
        valid = (r > msg.range_min) & (r < msg.range_max)
        # The points in base_link, less any inside the robot's own body.
        x = sx + r[valid] * np.cos(angle[valid])
        y = sy + r[valid] * np.sin(angle[valid])
        centre = np.hypot(x, y)
        keep = centre > self.robot_radius
        x, y, centre = x[keep], y[keep], centre[keep]
        # In the robot's path, forwards or backwards: within the corridor either side.
        # Clearance from the round body's edge.
        path = np.abs(y) <= self.corridor
        ahead, behind = path & (x > 0.0), path & (x <= 0.0)
        self.front_clearance = (float(centre[ahead].min()) - self.robot_radius
                                if ahead.any() else math.inf)
        self.rear_clearance = (float(centre[behind].min()) - self.robot_radius
                               if behind.any() else math.inf)
        self.front_arc = self.arc_clearance(x, y)
        # The points in odom too, for checking the line of sight to trail points.
        pose = self.robot_pose()
        if pose is not None:
            c, s_ = math.cos(pose[2]), math.sin(pose[2])
            self.scan_odom = np.column_stack((pose[0] + c * x - s_ * y,
                                              pose[1] + s_ * x + c * y))
        self.scan_time = self.now_s()
        # Straight back is at pi - syaw in the LiDAR's own frame.
        back = wrap_angle(math.pi - syaw)
        self.rear_visible = any(msg.angle_min - 1e-3 <= a <= msg.angle_max + 1e-3
                                for a in (back - 2 * math.pi, back, back + 2 * math.pi))

    def arc_clearance(self, x, y, step=0.05):
        """How far the robot can drive along its current arc (from the last command)
        before its body plus the side margin reaches a point; inf if it isn't turning."""
        v, w = self.linear, self.angular
        if v < 0.05 or abs(w) < 0.05 or len(x) == 0:
            return math.inf
        k = w / v                                      # curvature, 1/m
        reach = self.max_linear ** 2 / (2.0 * self.max_decel) + self.obstacle_slow + 0.2
        s = np.arange(step, reach + step, step)
        cx, cy = np.sin(k * s) / k, (1.0 - np.cos(k * s)) / k  # centre along the arc
        d = np.hypot(x[None, :] - cx[:, None], y[None, :] - cy[:, None])
        # Points already within the margin now are the straight check's business.
        fresh = np.hypot(x, y) >= self.corridor
        hit = (d < self.corridor) & fresh[None, :]
        if not hit.any():
            return math.inf
        first = np.argmax(hit, axis=0)[hit.any(axis=0)]
        return float(max(0.0, s[first.min()] - step))

    def clearances(self):
        """(front, rear) LiDAR clearance, less how far the robot has moved since the scan
        (and will move before the next command)."""
        if self.scan_time is None:
            return math.inf, math.inf
        moved = self.linear * (self.now_s() - self.scan_time + self.period + self.latency)
        front = min(self.front_clearance, self.front_arc)
        return front - max(moved, 0.0), self.rear_clearance - max(-moved, 0.0)

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time()).transform
        except tf2_ros.TransformException:
            return None
        return t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation)

    def publish(self, linear, angular):
        # Acceleration limit when speeding up, forwards or backwards; stopping is immediate.
        if linear > 0.0:
            linear = min(linear, max(self.linear, 0.0) + self.max_accel * self.period)
        elif linear < 0.0:
            linear = max(linear, min(self.linear, 0.0) - self.max_accel * self.period)
        self.linear = linear
        self.angular = angular
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)

    def log_state(self, why, state):
        """Set the state for feedback; log `why` when the state changes."""
        if state != self.logged_state:
            self.logged_state = state
            self.get_logger().info(why)
        self.state = state

    # --- enable / disable -----------------------------------------------------------

    def set_enabled(self, enabled, why):
        """Start or stop following."""
        if enabled == self.enabled:
            return
        self.enabled = enabled
        self.logged_state = None
        self.state = 'no_estimate'
        if not enabled:
            self.stop()
            self.state = 'disabled'
        self.get_logger().info(f"{'Following' if enabled else 'Stopped following'} ({why})")

    def on_enable(self, request, response):
        was = self.enabled
        self.set_enabled(request.data, '~/enable')
        response.success = True
        response.message = (f"{'Enabled' if request.data else 'Disabled'}"
                            + (' (no change)' if was == request.data else ''))
        return response

    # --- FollowPerson action -------------------------------------------------------

    async def execute(self, goal_handle):
        """Follow until the goal is cancelled, replaced, disabled or hits a limit.

        Waits without blocking the executor: action_tick() checks the goal five times a
        second, sends feedback, and completes goal['done'] when it ends."""
        request = goal_handle.request
        if self.goal is not None:
            self.get_logger().info('New FollowPerson goal replaces the current one')
            self.end_goal(self.goal, 'abort', 'Replaced by a new goal', disable=False)
        self.follow_distance = (self.safe_follow_distance(request.follow_distance)
                                if request.follow_distance > 0
                                else self.default_follow_distance)
        self.driven = 0.0
        goal = {'handle': goal_handle, 'request': request, 'start': self.now_s(),
                'done': Future()}
        self.goal = goal
        self.set_enabled(True, 'FollowPerson goal')
        self.get_logger().info(
            f'FollowPerson: {self.follow_distance:g} m behind'
            + (f', for {request.max_duration_s:g} s' if request.max_duration_s > 0 else '')
            + (f', giving up after {request.lost_timeout_s:g} s lost'
               if request.lost_timeout_s > 0 else ''))
        status, result = await goal['done']
        getattr(goal_handle, {'succeed': 'succeed', 'cancel': 'canceled',
                              'abort': 'abort'}[status])()
        return result

    def action_tick(self):
        """Check the active FollowPerson goal: end it, or send feedback."""
        goal = self.goal
        if goal is None:
            return
        handle, request, now = goal['handle'], goal['request'], self.now_s()
        seen = max(self.person_time or goal['start'], goal['start'])
        if handle.is_cancel_requested:
            self.end_goal(goal, 'cancel', 'Cancelled')
        elif not self.enabled:
            self.end_goal(goal, 'abort', 'Disabled with ~/enable')
        elif request.max_duration_s > 0 and now - goal['start'] >= request.max_duration_s:
            self.end_goal(goal, 'succeed', f'Followed for {request.max_duration_s:g} s')
        elif request.lost_timeout_s > 0 and now - seen >= request.lost_timeout_s:
            self.end_goal(goal, 'abort', f'Person lost for {request.lost_timeout_s:g} s')
        else:
            feedback = FollowPerson.Feedback()
            feedback.state = self.state
            feedback.distance_to_person = float(self.along)
            feedback.person_speed = float(math.hypot(*self.person_velocity))
            feedback.robot_speed = float(self.linear)
            handle.publish_feedback(feedback)

    def end_goal(self, goal, status, message, disable=True):
        """Finish a FollowPerson goal; the robot stops unless another goal takes over."""
        if self.goal is goal:
            self.goal = None
            self.follow_distance = self.default_follow_distance
            if disable:
                self.set_enabled(False, f'FollowPerson ended: {message}')
        result = FollowPerson.Result()
        result.duration_s = float(self.now_s() - goal['start'])
        result.distance_driven_m = float(self.driven)
        result.message = message
        goal['done'].set_result((status, result))

    # --- main loop -------------------------------------------------------------------

    def step(self):
        self.track_driven()
        if self.enabled:
            self.drive()

    def track_driven(self):
        pose = self.robot_pose()
        if pose is not None:
            if not self.history or math.dist(self.history[-1], pose[:2]) >= 0.25:
                self.history = self.history[-40:] + [pose[:2]]
            if self.last_xy is not None:
                step = math.dist(self.last_xy, pose[:2])
                self.driven += step
                if self.linear < 0.0:
                    self.backed += step
            self.last_xy = pose[:2]

    def drive(self):
        pose = self.robot_pose()
        if pose is None or self.person is None:
            self.state = 'no_estimate'
            self.publish(0.0, 0.0)
            return
        lost = self.now_s() - self.person_time > self.lost_after
        if lost != self.lost:
            self.lost = lost
            if lost:
                # Treat them as standing where they were last seen, and go there.
                self.person_velocity = (0.0, 0.0)
                self.backing = self.holding = False
                self.aside = None
                why = 'lost' if self.reliable else 'unreliable'
                self.get_logger().info(
                    f'Person estimate {why}; going to where they were last reliably seen '
                    f'({self.person[0]:.2f}, {self.person[1]:.2f} in odom)')
            else:
                self.get_logger().info('Person estimate back; following')
        rx, ry, ryaw = pose

        # Drop breadcrumbs the robot has reached or passed: everything before the nearest
        # one in the next 3 m of trail (not further, where a loop in the trail can come
        # back near the robot), and that one too once it is within reached_m.
        window, length = 0, 0.0
        while window < len(self.trail) and length < 3.0:
            if window:
                length += math.dist(self.trail[window - 1], self.trail[window])
            window += 1
        if window:
            near = min(range(window), key=lambda i: math.dist(self.trail[i], (rx, ry)))
            if math.dist(self.trail[near], (rx, ry)) < self.reached:
                near += 1
            del self.trail[:near]

        # Distance to the person along the trail (robot -> breadcrumbs -> person).
        points = [(rx, ry)] + self.trail + [self.person]
        along = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        self.along = along
        self.publish_trail()
        # Straight-line gap from the robot's edge to the person (the tag on their back).
        gap = math.dist((rx, ry), self.person) - self.robot_radius
        bearing = wrap_angle(math.atan2(self.person[1] - ry, self.person[0] - rx) - ryaw)
        face = clamp(self.k_angular * bearing, -self.max_angular, self.max_angular)
        if abs(bearing) < 0.35:
            face = 0.0  # close up the estimate's direction wobbles; don't chase it

        front, rear = self.clearances()
        if not self.lost:
            if self.step_aside((rx, ry, ryaw), front, gap):
                return
            if self.back_away(gap, face, front, rear):
                return
        if self.holding:
            if gap < self.person_stop + self.resume_margin:
                self.log_state(f'Backed away; waiting for the person to move off '
                               f'({gap:.2f} m)', 'waiting')
                self.publish(0.0, face)
                return
            self.holding = False

        if self.unstick_from is not None and self.unstick((rx, ry), rear):
            return

        if self.lost and (gap <= self.person_stop or along <= self.follow_distance):
            self.log_state('At the last place the person was seen; waiting for the signal',
                           'lost')
            self.publish(0.0, 0.0)
            return

        if gap <= self.person_stop:
            # 2 ft or closer: no forward motion, but keep facing the person.
            self.log_state(f'{gap:.2f} m from the person (stop at '
                           f'{self.person_stop:g} m); stopped', 'waiting')
            self.publish(0.0, face)
            return

        if along <= self.follow_distance:
            # Close enough: stop, but keep facing the person.
            bearing = wrap_angle(math.atan2(self.person[1] - ry, self.person[0] - rx) - ryaw)
            angular = clamp(self.k_angular * bearing, -self.max_angular, self.max_angular)
            # Close up the estimate's direction wobbles by 10-20 degrees; don't chase it.
            if abs(bearing) < 0.35:
                angular = 0.0
            self.log_state(f'Within {self.follow_distance:g} m of the person; waiting',
                           'waiting')
            self.publish(0.0, angular)
            return

        target, self.target_index = self.pick_target((rx, ry))
        heading_error = wrap_angle(math.atan2(target[1] - ry, target[0] - rx) - ryaw)
        angular = clamp(self.k_angular * heading_error, -self.max_angular, self.max_angular)

        if abs(heading_error) > self.turn_in_place:
            self.log_state('Turning towards the trail', 'turning')
            self.publish(0.0, angular)
            return
        if front < self.obstacle_stop:
            self.blocked(front, angular, (rx, ry))
            return
        self.blocked_since = None

        trail_gap = along - self.follow_distance
        # The person's pace in the direction the robot is heading for: negative when they
        # walk back towards it.
        aim = math.atan2(target[1] - ry, target[0] - rx)
        person_pace = (self.person_velocity[0] * math.cos(aim)
                       + self.person_velocity[1] * math.sin(aim))
        linear = clamp(person_pace + self.k_linear * trail_gap, 0.0, self.max_linear)
        linear = min(linear, self.speed_cap(trail_gap), self.person_cap(gap),
                     self.obstacle_cap(front),
                     self.braking_cap(trail_gap),
                     self.braking_cap(gap - self.person_stop),
                     self.braking_cap(front - self.obstacle_stop))
        if self.lost:
            linear = min(linear, self.lost_speed)
        linear *= max(0.0, math.cos(heading_error))
        if self.lost:
            self.log_state('Going to where the person was last seen', 'lost')
        else:
            self.log_state('Following', 'following')
        if linear > 0.0:
            self.backed = 0.0
        # Got clear: stuck_progress_m from where it last got stuck, so the tries start over.
        if self.unstick_tries and math.dist((rx, ry), self.stuck_at) >= self.stuck_progress:
            self.unstick_tries = 0
        self.publish(linear, angular)

    def pick_target(self, xy):
        """(point, trail index or None for the person) to steer for: the first trail point
        lookahead_m away, or the furthest point before it with a clear line of sight."""
        reach = len(self.trail)
        for i, point in enumerate(self.trail):
            if math.dist(point, xy) >= self.lookahead:
                reach = i
                break
        # Candidates, nearest first: trail points up to the lookahead one (or, if none is
        # that far, all of them and then the person).
        candidates = [(p, i) for i, p in enumerate(self.trail[:reach + 1])]
        if reach == len(self.trail):
            candidates.append((self.person, None))
        for point, index in reversed(candidates):
            if self.line_clear(xy, point):
                return point, index
        return candidates[0]

    def line_clear(self, a, b):
        """Whether the robot (its width plus the side margin) can drive straight from a to
        b without passing any point of the last LiDAR scan."""
        pts = self.scan_odom
        if len(pts) == 0:
            return True
        ax, ay = a
        dx, dy = b[0] - ax, b[1] - ay
        length2 = dx * dx + dy * dy
        if length2 < 1e-6:
            return True
        t = ((pts[:, 0] - ax) * dx + (pts[:, 1] - ay) * dy) / length2
        ahead = (t > 0.0) & (t <= 1.0)
        if not ahead.any():
            return True
        px, py = pts[ahead, 0] - (ax + t[ahead] * dx), pts[ahead, 1] - (ay + t[ahead] * dy)
        return bool((np.hypot(px, py) >= self.corridor).all())

    def blocked(self, front, angular, xy):
        """Something is in the way ahead: wait, and after stuck_after_s start backing up."""
        now = self.now_s()
        if self.blocked_since is None:
            self.blocked_since = now
        if now - self.blocked_since < self.stuck_after:
            self.log_state(f'Obstacle {front:.2f} m ahead; waiting', 'obstacle')
            self.publish(0.0, angular)
            return
        if self.unstick_tries >= self.stuck_attempts:
            self.log_state(f'Still blocked after {self.unstick_tries} tries to get unstuck; '
                           'waiting', 'stuck')
            self.publish(0.0, 0.0)
            return
        self.unstick_tries += 1
        self.stuck_at = xy
        self.unstick_from = xy
        # Skip the blocked trail point and the ones near it (within lookahead_m), which are
        # most likely behind the same obstacle.
        skip = 0
        if self.target_index is not None:
            blocked_at = self.trail[self.target_index]
            skip = self.target_index + 1
            while skip < len(self.trail) and \
                    math.dist(self.trail[skip], blocked_at) < self.lookahead:
                skip += 1
        self.unstick_skip = skip
        self.blocked_since = None
        self.get_logger().info(
            f'Blocked for {self.stuck_after:g} s; backing up {self.stuck_backup:g} m to get '
            f'unstuck (try {self.unstick_tries} of {self.stuck_attempts})')
        self.publish(0.0, 0.0)

    def unstick(self, xy, rear):
        """Back up to get unstuck; True while still backing up."""
        backed = math.dist(self.unstick_from, xy)
        why = None
        if backed >= self.stuck_backup:
            why = f'backed up {backed:.2f} m'
        elif rear - self.obstacle_stop < 0.02:
            why = f'something {rear:.2f} m behind after {backed:.2f} m'
        elif not self.rear_visible and self.backed >= self.blind_backup:
            why = f"the LiDAR can't see behind; backed {self.backed:.2f} m"
        if why is None:
            speed = min(self.backup_speed, rear - self.obstacle_stop,
                        max(0.05, self.stuck_backup - backed + 0.05))
            self.log_state('Backing up to get unstuck', 'unsticking')
            self.publish(-speed, 0.0)
            return True
        skipped = min(self.unstick_skip, len(self.trail))
        del self.trail[:skipped]
        self.unstick_from = None
        self.get_logger().info(f'Unstuck: {why}; skipped {skipped} trail point(s) and '
                               'aiming at the next')
        self.publish(0.0, 0.0)
        return False

    def in_the_way(self, xy):
        """(along, lateral, walking direction) of the robot relative to the person's line
        of walking, or None if the person isn't walking."""
        vx, vy = self.person_velocity
        speed = math.hypot(vx, vy)
        if speed < self.yield_speed:
            return None
        ux, uy = vx / speed, vy / speed
        rx, ry = xy[0] - self.person[0], xy[1] - self.person[1]
        return rx * ux + ry * uy, ux * ry - uy * rx, (ux, uy)

    def plan_aside(self, xy, lateral, u):
        """Where to go to get out of the way: to a side of the person's line (its own side
        first) if the LiDAR shows room, else ahead of them along it; None if neither."""
        x, y = xy
        ux, uy = u
        own = 1.0 if lateral >= 0.0 else -1.0
        # Prefer the side back towards where the robot came from (it followed the person
        # there, so they've been there and are less likely to walk there next); else its
        # own side of their line, the shorter way out.
        first = own
        back = next((h for h in reversed(self.history) if math.dist(h, xy) >= 1.0), None)
        if back is not None:
            towards = (back[0] - x) * -uy + (back[1] - y) * ux   # + : the left side
            if abs(towards) > 0.3:
                first = 1.0 if towards > 0 else -1.0
        if self.prefer_side is not None:                 # replanning: keep to its side
            first = self.prefer_side
        for side in (first, -first):
            nx, ny = -uy * side, ux * side
            need = self.yield_lateral - side * lateral
            if need <= self.yield_max and self.line_clear(
                    xy, (x + nx * (need + 0.1), y + ny * (need + 0.1))):
                return {'mode': 'side', 'dir': (nx, ny), 'side': side, 'need': need}
        if self.line_clear(xy, (x + ux * 1.2, y + uy * 1.2)):
            return {'mode': 'ahead', 'dir': (ux, uy), 'side': own, 'need': math.inf}
        return None

    def step_aside(self, pose, front, gap):
        """Get out of the way of a person walking towards the robot; True if it handled
        this step."""
        x, y, yaw = pose
        now = self.now_s()
        if gap < self.person_backup:
            # Already this close: no time to turn and drive; reversing (backing away) is
            # immediate.
            if self.aside is not None:
                self.get_logger().info('Person too close to step aside; backing away')
            self.aside = self.in_way_since = None
            return False
        rel = self.in_the_way((x, y))
        in_way = rel is not None and 0.0 < rel[0] < self.yield_ahead and \
            abs(rel[1]) < self.yield_lateral
        a = self.aside
        if a is not None and in_way and now - a['planned'] > 1.5:
            # They've turned well away from the line they were on and are coming at the
            # robot again: plan again (at most every 1.5 s, keeping its side if it can).
            ux, uy = a['u']
            if rel[2][0] * ux + rel[2][1] * uy < math.cos(1.0):
                self.prefer_side = a['side']
                a = self.aside = None
        if a is None:
            if not in_way:
                self.in_way_since = None
                return False
            if self.in_way_since is None:
                self.in_way_since = now
            if now - self.in_way_since < self.yield_confirm:
                return False                            # make sure it isn't velocity noise
            plan = self.plan_aside((x, y), rel[1], rel[2])
            self.prefer_side = None
            if plan is None:
                return False                            # no room: back away instead
            plan.update(u=rel[2], start=(x, y), since=now, planned=now)
            a = self.aside = plan
            where = ('ahead of them' if plan['mode'] == 'ahead' else
                     f"{'left' if plan['side'] > 0 else 'right'} of their path")
            self.get_logger().info(f'Person walking towards the robot ({rel[0]:.1f} m ahead '
                                   f'of them); stepping {where}')
        # Relative to the line they were walking (they may have paused).
        ux, uy = a['u']
        dx, dy = x - self.person[0], y - self.person[1]
        along, lateral = dx * ux + dy * uy, ux * dy - uy * dx
        # Done once they've walked past; or (aside) once they've stopped coming this way
        # and are well away; or timed out.
        far = a['mode'] == 'side' and gap > 2.5 and not in_way
        if along < -0.3 or far or now - a['since'] > self.yield_timeout:
            self.aside = None
            self.get_logger().info('Out of the person\'s way; following again')
            return False
        moved = math.dist(a['start'], (x, y))
        if a['mode'] == 'ahead':
            # Keep ahead of them until a side opens up, then step aside there.
            plan = self.plan_aside((x, y), lateral, a['u'])
            if plan is not None and plan['mode'] == 'side':
                plan.update(u=a['u'], start=(x, y), since=a['since'], planned=now)
                a = self.aside = plan
                moved = 0.0
                self.get_logger().info('Room to the side now; stepping aside')
        if a['mode'] == 'side' and (a['side'] * lateral >= self.yield_lateral
                                    or moved >= min(a['need'], self.yield_max)):
            self.log_state('Stepped aside; waiting for the person to pass', 'aside')
            self.publish(0.0, 0.0)
            return True
        heading = math.atan2(a['dir'][1], a['dir'][0])
        error = wrap_angle(heading - yaw)
        angular = clamp(self.k_angular * error, -self.max_angular, self.max_angular)
        if abs(error) > 0.8 or front < self.obstacle_stop:
            # Turn first (a round base sweeps nothing new); or wait if something's ahead.
            self.log_state('Turning to get out of the way', 'aside')
            self.publish(0.0, angular if abs(error) > 0.05 else 0.0)
            return True
        top = 0.3 if a['mode'] == 'side' else 0.5       # ahead: faster than they walk
        speed = min(top, self.max_linear, self.obstacle_cap(front),
                    self.braking_cap(front - self.obstacle_stop))
        self.log_state('Getting out of the way', 'aside')
        self.publish(speed * max(0.0, math.cos(error)), angular)
        return True

    def back_away(self, gap, face, front, rear):
        """Reverse away from a person who is too close; True if it handled this step."""
        # Something touching-close ahead with the person this near is most likely them.
        person_near = gap < self.person_stop + self.resume_margin
        crowded = front < self.obstacle_stop and person_near
        if not self.backing:
            if gap >= self.person_backup and not crowded:
                return False
            self.backing = True
            self.get_logger().info(
                f'Person {gap:.2f} m away, something {front:.2f} m ahead; backing away')
        # Keep going until 2 ft from the person and a little clear ahead.
        short = max(self.person_stop - gap,
                    (self.obstacle_stop + 0.1 - front) if person_near else 0.0)
        if short <= 0.0:
            self.backing = False
            self.holding = True
            self.get_logger().info(f'Backed away: {gap:.2f} m from the person')
            self.publish(0.0, face)
            return True
        # Slow down near anything behind (0.1 m/s at 0.1 m short of obstacle_stop_m) and
        # stop once that is under 2 cm/s.
        if rear - self.obstacle_stop < 0.02:
            self.log_state(f'Something {rear:.2f} m behind; can\'t back away', 'blocked')
            self.publish(0.0, face)
            return True
        if not self.rear_visible and self.backed >= self.blind_backup:
            self.log_state(f"The LiDAR can't see behind; backed {self.backed:.2f} m, "
                           'not reversing further', 'blocked')
            self.publish(0.0, face)
            return True
        speed = min(clamp(short + 0.05, 0.05, self.backup_speed), rear - self.obstacle_stop)
        self.log_state('Backing away from the person', 'backing')
        self.publish(-speed, face)
        return True

    def obstacle_cap(self, clearance):
        """Top speed with an object clearance metres ahead of the robot's edge."""
        if clearance >= self.obstacle_slow:
            return self.max_linear
        fraction = (clearance - self.obstacle_stop) / (self.obstacle_slow - self.obstacle_stop)
        return self.creep_speed + max(0.0, fraction) * (self.max_linear - self.creep_speed)

    def braking_cap(self, distance):
        """Top speed that can still stop within distance metres."""
        return math.sqrt(2.0 * self.max_decel * max(distance, 0.0))

    def person_cap(self, gap):
        """Top speed with the robot's edge gap metres from the person, straight-line."""
        if gap <= self.person_stop:
            return 0.0
        if gap >= self.person_slow:
            return self.max_linear
        return self.max_linear * (gap - self.person_stop) / (self.person_slow - self.person_stop)

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
        rclpy.spin(node)   # single-threaded; see the comment in __init__
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
