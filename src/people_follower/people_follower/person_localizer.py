"""Place each detected person relative to the robot, from their camera box and the depth image.

For every box from person_detector:
  1. Depth: the depth image (same camera, same timestamp) inside the torso part of the
     box (the middle torso_width of its width, torso_top..torso_bottom of its height).
     The node takes the median (depth_percentile) of the valid pixels: the torso area
     keeps clear of the box edges, which often catch the wall behind the person, and a
     lower percentile reads short once the depth is noisy (sensor_noise).
  2. 3D point: the box centre column and the torso centre row, back-projected with
     the camera intrinsics from camera_info, at that depth. The camera sees the
     person's surface, so the point is pushed body_radius further along the
     horizontal ray to the body's centre.
  3. Robot frame: transformed into base_frame with TF.

Subscribes:
    /person/detections               vision_msgs/Detection2DArray (person_detector)
    /oakd/rgb/preview/depth          sensor_msgs/Image, 32FC1 metres (16UC1 mm also works)
    /oakd/rgb/preview/camera_info    sensor_msgs/CameraInfo
Publishes:
    /person/detections_3d   vision_msgs/Detection3DArray in base_link: one per box with a
                            usable depth; id = track ID, bbox centre = body centre at the
                            torso height, bbox size = the box's real-world width and height
    /person/markers         visualization_msgs/MarkerArray for RViz
"""

import math

from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
from vision_msgs.msg import (Detection2DArray, Detection3D, Detection3DArray,
                             ObjectHypothesisWithPose)
from visualization_msgs.msg import Marker, MarkerArray


def rotation_matrix(q):
    """3x3 rotation from a geometry_msgs Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class PersonLocalizer(Node):

    def __init__(self):
        super().__init__('person_localizer')
        self.declare_parameter('depth_topic', '/oakd/rgb/preview/depth')
        self.declare_parameter('camera_info_topic', '/oakd/rgb/preview/camera_info')
        self.declare_parameter('base_frame', 'base_link')
        # The images' own frame_id is Gazebo's sensor frame; TF has the standard optical
        # frame (z forward, x right, y down) at the same place, so points are put in that.
        self.declare_parameter('camera_optical_frame', 'oakd_rgb_camera_optical_frame')
        self.declare_parameter('torso_width', 0.3)       # fraction of the box width
        self.declare_parameter('torso_top', 0.2)         # fractions of the box height, from its top
        self.declare_parameter('torso_bottom', 0.55)
        self.declare_parameter('depth_percentile', 50.0)
        self.declare_parameter('min_pixels', 10)         # fewer valid depth pixels: skip the box
        self.declare_parameter('body_radius', 0.2)       # m from the body's surface to its centre
        self.declare_parameter('max_depth', 20.0)        # m

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.base_frame = p('base_frame')
        self.optical_frame = p('camera_optical_frame')
        self.torso_width = p('torso_width')
        self.torso_top = p('torso_top')
        self.torso_bottom = p('torso_bottom')
        self.percentile = p('depth_percentile')
        self.min_pixels = p('min_pixels')
        self.body_radius = p('body_radius')
        self.max_depth = p('max_depth')

        self.bridge = CvBridge()
        self.intrinsics = None     # fx, fy, cx, cy
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, p('camera_info_topic'), self.on_info,
                                 qos_profile_sensor_data)
        det_sub = message_filters.Subscriber(self, Detection2DArray, '/person/detections')
        depth_sub = message_filters.Subscriber(self, Image, p('depth_topic'),
                                               qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [det_sub, depth_sub], queue_size=10, slop=0.02)
        self.sync.registerCallback(self.on_detections)

        self.det_pub = self.create_publisher(Detection3DArray, '/person/detections_3d', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/person/markers', 10)
        self.get_logger().info(
            f'Placing people from {p("depth_topic")} into {self.base_frame}')

    def on_info(self, msg):
        k = msg.k
        self.intrinsics = (k[0], k[4], k[2], k[5])

    def camera_to_base(self, stamp):
        """(rotation, translation) from the optical frame into base_frame, or None."""
        for when in (Time.from_msg(stamp), Time()):
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, self.optical_frame, when,
                    timeout=Duration(seconds=0.05)).transform
            except tf2_ros.TransformException:
                continue
            return rotation_matrix(t.rotation), np.array(
                [t.translation.x, t.translation.y, t.translation.z])
        return None

    def torso_depth(self, depth, box):
        """Low-percentile depth (m) of the box's torso area, and its centre row; or None."""
        h_img, w_img = depth.shape
        cx, cy = box.center.position.x, box.center.position.y
        w, h = box.size_x, box.size_y
        top = cy - h / 2
        x0 = int(max(0, cx - w * self.torso_width / 2))
        x1 = int(min(w_img, cx + w * self.torso_width / 2 + 1))
        y0 = int(max(0, top + h * self.torso_top))
        y1 = int(min(h_img, top + h * self.torso_bottom + 1))
        if x1 <= x0 or y1 <= y0:
            return None
        patch = depth[y0:y1, x0:x1].astype(float).ravel()
        patch = patch[np.isfinite(patch) & (patch > 0.0) & (patch < self.max_depth)]
        if len(patch) < self.min_pixels:
            return None
        return float(np.percentile(patch, self.percentile)), (y0 + y1) / 2

    def on_detections(self, detections, depth_msg):
        if self.intrinsics is None:
            return
        depth = self.bridge.imgmsg_to_cv2(depth_msg)
        if depth_msg.encoding == '16UC1':
            depth = depth.astype(float) / 1000.0
        transform = self.camera_to_base(detections.header.stamp)
        if transform is None:
            return
        rotation, translation = transform
        fx, fy, cx, cy = self.intrinsics

        out = Detection3DArray()
        out.header.stamp = detections.header.stamp
        out.header.frame_id = self.base_frame
        for det in detections.detections:
            found = self.torso_depth(depth, det.bbox)
            if found is None:
                continue
            z, row = found
            u = det.bbox.center.position.x
            # Optical frame: x right, y down, z forward. Push the surface point back by the
            # body radius along the horizontal (x, z) direction of the ray.
            x = (u - cx) / fx * z
            y = (row - cy) / fy * z
            horizontal = math.hypot(x, z)
            scale = (horizontal + self.body_radius) / horizontal
            point = rotation @ np.array([x * scale, y, z * scale]) + translation

            d3 = Detection3D()
            d3.header = out.header
            d3.id = det.id
            d3.bbox.center.position.x, d3.bbox.center.position.y, d3.bbox.center.position.z = (
                float(point[0]), float(point[1]), float(point[2]))
            d3.bbox.center.orientation.w = 1.0
            d3.bbox.size.x = d3.bbox.size.y = det.bbox.size_x / fx * z
            d3.bbox.size.z = det.bbox.size_y / fy * z
            for result in det.results:
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis = result.hypothesis
                d3.results.append(hypothesis)
            out.detections.append(d3)
        self.det_pub.publish(out)
        self.publish_markers(out)

    def publish_markers(self, detections):
        markers = MarkerArray()
        clear = Marker()
        clear.header = detections.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, det in enumerate(detections.detections):
            pos = det.bbox.center.position
            body = Marker()
            body.header = detections.header
            body.ns, body.id, body.type = 'person', i, Marker.CYLINDER
            body.pose.position.x, body.pose.position.y, body.pose.position.z = pos.x, pos.y, 0.9
            body.pose.orientation.w = 1.0
            body.scale.x = body.scale.y = 0.5
            body.scale.z = 1.8
            body.color.r, body.color.g, body.color.b, body.color.a = 0.2, 0.6, 1.0, 0.5
            body.lifetime = Duration(seconds=0.5).to_msg()
            label = Marker()
            label.header = detections.header
            label.ns, label.id, label.type = 'person_id', i, Marker.TEXT_VIEW_FACING
            label.pose.position.x, label.pose.position.y, label.pose.position.z = pos.x, pos.y, 2.0
            label.pose.orientation.w = 1.0
            label.scale.z = 0.3
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'person {det.id}' if det.id else 'person'
            label.lifetime = Duration(seconds=0.5).to_msg()
            markers.markers += [body, label]
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = PersonLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
