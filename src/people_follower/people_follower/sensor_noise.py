"""Make the simulated camera and person detections behave more like real hardware.

Sits between Gazebo and the person-detection nodes (person_detector.launch.py with
noise:=true wires it in). Every effect has a parameter; set it to 0 to turn it off.
Defaults and explanations: config/sensor_noise.yaml.

Depth (OAK-D stereo): depth comes from disparity d = fx * baseline / z, so the noise and
rounding are applied to disparity and converted back. That makes the error grow with
the square of the distance, as on the real camera. The noise is mostly spatially
correlated (smooth blobs, like stereo matching errors), so averaging over a patch
doesn't remove it. Pixels next to depth edges and a few random ones get no depth (NaN).

Colour image: Gaussian pixel noise, random brightness and contrast per frame, and
horizontal motion blur from the robot's turn rate (/odom) and the exposure time.

Timing: the camera images are thinned to max_rate_hz, frames are dropped at random
(colour and depth together), and both are published latency_s late, keeping their
capture timestamps.

Detections: each person box can be dropped, its centre and size jittered, and a false
box (a random patch of the image) added now and then.

Topics (in -> out):
    /oakd/rgb/preview/image_raw  -> /oakd_noisy/rgb/preview/image_raw
    /oakd/rgb/preview/depth      -> /oakd_noisy/rgb/preview/depth
    /person/detections_clean     -> /person/detections   (from person_detector)
"""

import collections
import math

from cv_bridge import CvBridge
import cv2
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class SensorNoise(Node):

    def __init__(self):
        super().__init__('sensor_noise')
        params = {
            'image_in': '/oakd/rgb/preview/image_raw',
            'image_out': '/oakd_noisy/rgb/preview/image_raw',
            'depth_in': '/oakd/rgb/preview/depth',
            'depth_out': '/oakd_noisy/rgb/preview/depth',
            'camera_info': '/oakd/rgb/preview/camera_info',
            'detections_in': '/person/detections_clean',
            'detections_out': '/person/detections',
            'seed': 0,                          # 0: different every run
            # depth
            'baseline_m': 0.075,
            'disparity_sigma_px': 0.1,
            'correlated_fraction': 0.8,
            'correlation_px': 8,
            'subpixel_step_px': 0.125,
            'edge_jump_m': 0.25,
            'edge_hole_px': 1,
            'random_hole_fraction': 0.02,
            'min_depth_m': 0.35,
            'max_depth_m': 15.0,
            # colour image
            'pixel_sigma': 5.0,
            'brightness_range': 0.15,
            'contrast_range': 0.10,
            'exposure_s': 0.02,
            # timing
            'max_rate_hz': 15.0,
            'frame_drop_prob': 0.02,
            'latency_s': 0.1,
            # detections
            'detection_drop_prob': 0.05,
            'box_jitter_px': 2.0,
            'false_positive_prob': 0.01,
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in params}
        p = self.p

        seed = p['seed'] or None
        self.rng = np.random.default_rng(seed)
        self.seed = seed if seed is not None else int(self.rng.integers(1 << 31))
        self.bridge = CvBridge()
        self.fx = None
        self.turn_rate = 0.0
        self.last_kept = {'image': None, 'depth': None}
        self.queue = collections.deque()    # (release_ns, publisher, msg), in arrival order
        self.counts = {'image': [0, 0], 'depth': [0, 0]}

        self.image_pub = self.create_publisher(Image, p['image_out'], qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, p['depth_out'], qos_profile_sensor_data)
        self.det_pub = self.create_publisher(Detection2DArray, p['detections_out'], 10)
        self.create_subscription(Image, p['image_in'], self.on_image, qos_profile_sensor_data)
        self.create_subscription(Image, p['depth_in'], self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p['camera_info'], self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, p['detections_in'], self.on_detections, 10)
        self.create_timer(0.005, self.release)
        self.create_timer(10.0, self.log_counts)
        self.get_logger().info(
            f"Depth noise {p['disparity_sigma_px']:g} px disparity (baseline "
            f"{p['baseline_m']:g} m), image noise {p['pixel_sigma']:g}, up to "
            f"{p['max_rate_hz']:g} Hz, {p['latency_s'] * 1000:.0f} ms late, detections dropped "
            f"{p['detection_drop_prob'] * 100:g}% / false {p['false_positive_prob'] * 100:g}%; "
            f"seed {self.seed}")

    # --- inputs ------------------------------------------------------------------

    def on_info(self, msg):
        self.fx = msg.k[0]

    def on_odom(self, msg):
        self.turn_rate = msg.twist.twist.angular.z

    def keep_frame(self, stream, stamp):
        """Same decision for colour and depth: they share capture timestamps."""
        t = stamp_ns(stamp)
        rate = self.p['max_rate_hz']
        last = self.last_kept[stream]
        if rate > 0 and last is not None and t - last < 0.9e9 / rate:
            return False
        # Drop decision from the timestamp, so both streams drop the same frames.
        if np.random.default_rng([self.seed, t]).random() < self.p['frame_drop_prob']:
            return False
        self.last_kept[stream] = t
        return True

    def schedule(self, publisher, msg):
        release = self.get_clock().now().nanoseconds + int(self.p['latency_s'] * 1e9)
        self.queue.append((release, publisher, msg))

    def release(self):
        now = self.get_clock().now().nanoseconds
        while self.queue and self.queue[0][0] <= now:
            _, publisher, msg = self.queue.popleft()
            publisher.publish(msg)

    # --- colour image ------------------------------------------------------------

    def on_image(self, msg):
        self.counts['image'][0] += 1
        if not self.keep_frame('image', msg.header.stamp):
            return
        self.counts['image'][1] += 1
        img = self.bridge.imgmsg_to_cv2(msg).astype(np.float32)
        p = self.p
        if p['exposure_s'] > 0 and self.fx:
            length = abs(self.turn_rate) * p['exposure_s'] * self.fx
            if length >= 1.5:
                n = int(round(length))
                img = cv2.filter2D(img, -1, np.full((1, n), 1.0 / n, np.float32))
        gain = 1.0 + self.rng.uniform(-p['contrast_range'], p['contrast_range'])
        offset = 255.0 * self.rng.uniform(-p['brightness_range'], p['brightness_range'])
        img = (img - 128.0) * gain + 128.0 + offset
        if p['pixel_sigma'] > 0:
            img += self.rng.normal(0.0, p['pixel_sigma'], img.shape).astype(np.float32)
        out = self.bridge.cv2_to_imgmsg(np.clip(img, 0, 255).astype(np.uint8),
                                        encoding=msg.encoding)
        out.header = msg.header
        self.schedule(self.image_pub, out)

    # --- depth -------------------------------------------------------------------

    def correlated_noise(self, shape):
        """Unit-variance noise that is smooth over about correlation_px pixels."""
        h, w = shape
        c = max(1, int(self.p['correlation_px']))
        coarse = self.rng.normal(size=(h // c + 2, w // c + 2)).astype(np.float32)
        smooth = cv2.resize(coarse, (w + 2 * c, h + 2 * c), interpolation=cv2.INTER_CUBIC)
        smooth = smooth[c:c + h, c:c + w]
        return smooth / max(float(smooth.std()), 1e-6)

    def on_depth(self, msg):
        self.counts['depth'][0] += 1
        if not self.keep_frame('depth', msg.header.stamp) or not self.fx:
            return
        self.counts['depth'][1] += 1
        p = self.p
        depth = self.bridge.imgmsg_to_cv2(msg).astype(np.float32)
        if msg.encoding == '16UC1':
            depth /= 1000.0
        valid = np.isfinite(depth) & (depth > 0)
        focal_baseline = self.fx * p['baseline_m']

        disparity = np.zeros_like(depth)
        disparity[valid] = focal_baseline / depth[valid]
        a = math.sqrt(min(max(p['correlated_fraction'], 0.0), 1.0))
        noise = (a * self.correlated_noise(depth.shape)
                 + math.sqrt(1.0 - a * a) * self.rng.normal(size=depth.shape).astype(np.float32))
        disparity += p['disparity_sigma_px'] * noise
        if p['subpixel_step_px'] > 0:
            disparity = np.round(disparity / p['subpixel_step_px']) * p['subpixel_step_px']
        valid &= disparity > 0
        noisy = np.full_like(depth, np.nan)
        noisy[valid] = focal_baseline / disparity[valid]

        holes = ~valid | (noisy < p['min_depth_m']) | (noisy > p['max_depth_m'])
        if p['edge_jump_m'] > 0:
            filled = np.nan_to_num(depth, nan=p['max_depth_m'], posinf=p['max_depth_m'])
            edges = np.zeros(depth.shape, bool)
            edges[:, 1:] |= np.abs(np.diff(filled, axis=1)) > p['edge_jump_m']
            edges[1:, :] |= np.abs(np.diff(filled, axis=0)) > p['edge_jump_m']
            if p['edge_hole_px'] > 0:
                k = 2 * int(p['edge_hole_px']) + 1
                edges = cv2.dilate(edges.astype(np.uint8), np.ones((k, k), np.uint8)) > 0
            holes |= edges
        holes |= self.rng.random(depth.shape) < p['random_hole_fraction']
        noisy[holes] = np.nan

        out = self.bridge.cv2_to_imgmsg(noisy, encoding='32FC1')
        out.header = msg.header
        self.schedule(self.depth_pub, out)

    # --- detections --------------------------------------------------------------

    def on_detections(self, msg):
        p = self.p
        out = Detection2DArray()
        out.header = msg.header
        for det in msg.detections:
            if self.rng.random() < p['detection_drop_prob']:
                continue
            j = p['box_jitter_px']
            if j > 0:
                det.bbox.center.position.x += float(self.rng.normal(0, j))
                det.bbox.center.position.y += float(self.rng.normal(0, j))
                det.bbox.size_x = max(2.0, det.bbox.size_x + float(self.rng.normal(0, j)))
                det.bbox.size_y = max(2.0, det.bbox.size_y + float(self.rng.normal(0, j)))
            out.detections.append(det)
        if self.rng.random() < p['false_positive_prob']:
            out.detections.append(self.false_box())
        self.det_pub.publish(out)

    def false_box(self):
        """A person-shaped box on a random part of a 320x240 image."""
        det = Detection2D()
        w = float(self.rng.uniform(15, 60))
        h = w * float(self.rng.uniform(2.0, 3.0))
        det.bbox.center.position.x = float(self.rng.uniform(w / 2, 320 - w / 2))
        det.bbox.center.position.y = float(self.rng.uniform(h / 2, max(h / 2 + 1, 240 - h / 2)))
        det.bbox.size_x, det.bbox.size_y = w, h
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = 'person'
        hypothesis.hypothesis.score = float(self.rng.uniform(0.4, 0.6))
        det.results.append(hypothesis)
        return det

    def log_counts(self):
        text = ', '.join(f'{k} {kept}/{seen} frames kept'
                         for k, (seen, kept) in self.counts.items())
        self.get_logger().info(text)
        self.counts = {'image': [0, 0], 'depth': [0, 0]}


def main():
    rclpy.init()
    node = SensorNoise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
