#!/usr/bin/env python3
"""Score a person-following run against Gazebo's true poses (used by test_two_people.sh).

Reads the true poses of the robot and the people from Gazebo
(/world/<world>/dynamic_pose/info), and the follower's chosen target from
/person/target (base_link). While the run lasts it:
  - starts the second person walking (a command given with --start-cmd) once the
    target person comes within --start-radius of --start-at,
  - ends the run once the target reaches the end area (--end-x-below, --end-y-below),
    or after --timeout seconds.
Then it reports how far behind the robot stayed, who the follower's target really was
over time (the target person, the other person, or neither), every switch, the
closest approach to each person, and the bumps, with what was nearest at each bump.
"""

import argparse
import bisect
import json
import math
import signal
import subprocess
import threading
import time

from geometry_msgs.msg import PointStamped
from irobot_create_msgs.msg import HazardDetectionVector
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def yaw_from(o):
    return math.atan2(2 * (o.get('w', 1) * o.get('z', 0) + o.get('x', 0) * o.get('y', 0)),
                      1 - 2 * (o.get('y', 0) ** 2 + o.get('z', 0) ** 2))


class GazeboPoses:
    """True poses of named models from Gazebo's dynamic_pose/info, by sim time."""

    def __init__(self, world, names):
        self.times = {n: [] for n in names}
        self.poses = {n: [] for n in names}
        self.proc = subprocess.Popen(
            ['gz', 'topic', '-e', '--json-output', '-t', f'/world/{world}/dynamic_pose/info'],
            stdout=subprocess.PIPE, text=True)
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        for line in self.proc.stdout:
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            stamp = msg.get('header', {}).get('stamp', {})
            t = int(stamp.get('sec', 0)) + int(stamp.get('nsec', 0)) * 1e-9
            for pose in msg.get('pose', []):
                name = pose.get('name')
                if name in self.times:
                    p = pose.get('position', {})
                    self.times[name].append(t)
                    self.poses[name].append(
                        (p.get('x', 0.0), p.get('y', 0.0), yaw_from(pose.get('orientation', {}))))

    def at(self, name, t=None):
        times = self.times[name]
        if not times:
            return None
        i = len(times) - 1 if t is None else min(bisect.bisect_left(times, t), len(times) - 1)
        return self.poses[name][i]

    def in_robot_frame(self, robot, name, t):
        r, p = self.at(robot, t), self.at(name, t)
        if r is None or p is None:
            return None
        dx, dy = p[0] - r[0], p[1] - r[1]
        c, s = math.cos(r[2]), math.sin(r[2])
        return c * dx + s * dy, -s * dx + c * dy

    def stop(self):
        self.proc.kill()


class FollowTest(Node):

    def __init__(self, args):
        super().__init__('follow_test', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.args = args
        self.people = [args.target] + ([args.other] if args.other else [])
        self.gz = GazeboPoses(args.world, [args.robot] + self.people)
        self.samples = []        # (t, robot-target dist, robot-other dist or None)
        self.followed = []       # (t, who)
        self.bumps = []          # (t, nearest description)
        self.bumping = False
        self.started_other = None
        self.ended = None
        self.hazard_msgs = 0
        self.create_subscription(PointStamped, '/person/target', self.on_target, 10)
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.on_hazard,
                                 qos_profile_sensor_data)
        self.create_timer(0.1, self.sample)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_target(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        best, best_d = 'neither', self.args.match_radius
        for name in self.people:
            truth = self.gz.in_robot_frame(self.args.robot, name, t)
            if truth is not None:
                d = math.dist((msg.point.x, msg.point.y), truth)
                if d < best_d:
                    best, best_d = name, d
        self.followed.append((t, best))

    def on_hazard(self, msg):
        self.hazard_msgs += 1
        bump = any(h.type == 1 for h in msg.detections)
        if bump and not self.bumping:
            robot = self.gz.at(self.args.robot)
            near = []
            for name in self.people:
                p = self.gz.at(name)
                if robot and p:
                    near.append((math.dist(robot[:2], p[:2]), name))
            near.sort()
            what = (f'{near[0][1]} {near[0][0]:.2f} m away (centres)' if near else '?')
            pos = f'robot at ({robot[0]:.1f}, {robot[1]:.1f})' if robot else ''
            self.bumps.append((self.now(), f'{pos}, nearest person {what}'))
        self.bumping = bump

    def sample(self):
        t = self.now()
        robot, target = self.gz.at(self.args.robot), self.gz.at(self.args.target)
        if robot is None or target is None:
            return
        other = self.gz.at(self.args.other) if self.args.other else None
        self.samples.append((t, math.dist(robot[:2], target[:2]),
                             math.dist(robot[:2], other[:2]) if other else None))
        a = self.args
        if (a.start_cmd and self.started_other is None
                and math.dist(target[:2], a.start_at) < a.start_radius):
            self.started_other = t
            subprocess.Popen(a.start_cmd, shell=True, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            print(f'[{t:.0f} s] target reached ({a.start_at[0]:g}, {a.start_at[1]:g}): '
                  f'second person starts walking', flush=True)
        if target[0] < a.end_x_below and target[1] < a.end_y_below and self.ended is None:
            self.ended = t
            print(f'[{t:.0f} s] target reached the end area: test over', flush=True)

    def report(self, interrupted=False):
        def pct(v, q):
            v = sorted(v)
            return v[min(len(v) - 1, int(q * len(v)))] if v else float('nan')
        d = [s[1] for s in self.samples]
        print('\n=== Results ===')
        if self.samples:
            print(f'run length {self.samples[-1][0] - self.samples[0][0]:.0f} s (sim time)')
            print(f'robot to target person (centres): median {pct(d, .5):.2f} m, '
                  f'10% {pct(d, .1):.2f}, 90% {pct(d, .9):.2f}, '
                  f'min {min(d):.2f}, max {max(d):.2f} m')
            o = [s[2] for s in self.samples if s[2] is not None]
            if o:
                print(f'closest the robot came to the other person: {min(o):.2f} m (centres)')
        if self.followed:
            counts = {}
            for _, who in self.followed:
                counts[who] = counts.get(who, 0) + 1
            total = len(self.followed)
            print('follower target was: ' + ', '.join(
                f'{who} {100 * n / total:.0f}%' for who, n in sorted(counts.items())))
            switches, prev = [], None
            for t, who in self.followed:
                if who != prev:
                    switches.append((t, who))
                    prev = who
            print(f'changes of who the target is ({len(switches) - 1}):')
            for t, who in switches[1:]:
                print(f'  [{t:.0f} s] now {who}')
        else:
            print('follower never had a target')
        print(f'bumps: {len(self.bumps)} (hazard_detection messages: {self.hazard_msgs})')
        for t, what in self.bumps:
            print(f'  [{t:.0f} s] {what}')
        if self.started_other is None and self.args.start_cmd:
            print('the second person never started (target never reached the start point)')
        print('test ended by: ' + ('the target reaching the end area' if self.ended
                                   else 'being stopped' if interrupted else 'the time limit'))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--world', default='simple_colored_warehouse')
    parser.add_argument('--robot', default='turtlebot4')
    parser.add_argument('--target', default='human')
    parser.add_argument('--other', default='human2')
    parser.add_argument('--start-at', type=float, nargs=2, default=(-3.5, 6.5))
    parser.add_argument('--start-radius', type=float, default=0.6)
    parser.add_argument('--start-cmd', default='')
    parser.add_argument('--end-x-below', type=float, default=-2.0)
    parser.add_argument('--end-y-below', type=float, default=-2.0)
    parser.add_argument('--match-radius', type=float, default=1.0,
                        help='the target counts as a person when within this of their true '
                             'position (m)')
    parser.add_argument('--timeout', type=float, default=420.0)
    args = parser.parse_args()

    # Stop cleanly (and still print the report) on Ctrl+C or a plain kill. Ctrl+C is
    # re-enabled because background jobs of a script start with it ignored.
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)

    rclpy.init()
    node = FollowTest(args)
    deadline = time.time() + args.timeout
    interrupted = False
    try:
        while time.time() < deadline and node.ended is None:
            rclpy.spin_once(node, timeout_sec=0.05)
        settle = time.time() + 2.0      # let the last messages in
        while time.time() < settle:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        interrupted = True
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    node.report(interrupted)
    node.gz.stop()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
