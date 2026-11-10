"""Detect and track people in the robot's camera image with YOLO (Ultralytics).

Runs a COCO-pretrained YOLO model on the OAK-D colour image, keeps only the
"person" class (COCO class 0), and gives each person a track ID that stays the same
from frame to frame (ByteTrack). Needs the Python environment from
scripts/setup_yolo.sh; person_detector.launch.py starts it with that Python.

Subscribes:
    /oakd/rgb/preview/image_raw   sensor_msgs/Image (320x240 in simulation)
Publishes:
    /person/detections            vision_msgs/Detection2DArray (topic: detections_topic),
                                  one Detection2D per person: bbox centre and size in
                                  pixels, id = track ID ('' until the tracker confirms
                                  it), and one result with class_id 'person' and the
                                  confidence
    /person/image_annotated       sensor_msgs/Image with the boxes drawn (for RViz/rqt)
"""

import time

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

PERSON_CLASS = 0  # COCO


class PersonDetector(Node):

    def __init__(self):
        super().__init__('person_detector')
        self.declare_parameter('model', 'yolo26n.pt')     # path to the .pt file
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('detections_topic', '/person/detections')
        self.declare_parameter('confidence', 0.4)        # drop weaker detections
        self.declare_parameter('image_size', 320)        # YOLO input size (camera is 320x240)
        self.declare_parameter('device', '')             # '' = GPU if available, else 'cpu'
        self.declare_parameter('tracker', 'bytetrack.yaml')
        self.declare_parameter('publish_annotated', True)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.confidence = p('confidence')
        self.image_size = p('image_size')
        self.device = p('device') or None
        self.tracker = p('tracker')
        self.publish_annotated = p('publish_annotated')

        self.model = YOLO(p('model'))
        self.bridge = CvBridge()
        self.stats = {'frames': 0, 'with_person': 0, 'infer_s': 0.0, 'since': time.monotonic()}

        self.create_subscription(Image, p('image_topic'), self.on_image, qos_profile_sensor_data)
        self.det_pub = self.create_publisher(Detection2DArray, p('detections_topic'), 10)
        self.img_pub = self.create_publisher(Image, '/person/image_annotated', 10)
        self.create_timer(5.0, self.log_stats)
        self.get_logger().info(
            f"Model {p('model')}, confidence >= {self.confidence:g}, input {self.image_size} px, "
            f"tracker {self.tracker}, listening on {p('image_topic')}")

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        start = time.monotonic()
        result = self.model.track(
            frame, persist=True, tracker=self.tracker, classes=[PERSON_CLASS],
            conf=self.confidence, imgsz=self.image_size, device=self.device, verbose=False)[0]
        self.stats['infer_s'] += time.monotonic() - start
        self.stats['frames'] += 1

        out = Detection2DArray()
        out.header = msg.header
        boxes = result.boxes
        ids = boxes.id.int().tolist() if boxes.id is not None else [None] * len(boxes)
        for (cx, cy, w, h), score, track_id in zip(
                boxes.xywh.tolist(), boxes.conf.tolist(), ids):
            det = Detection2D()
            det.header = msg.header
            det.id = '' if track_id is None else str(track_id)
            det.bbox.center.position.x, det.bbox.center.position.y = cx, cy
            det.bbox.size_x, det.bbox.size_y = w, h
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'person'
            hypothesis.hypothesis.score = float(score)
            det.results.append(hypothesis)
            out.detections.append(det)
        if out.detections:
            self.stats['with_person'] += 1
        self.det_pub.publish(out)

        if self.publish_annotated:
            annotated = self.bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8')
            annotated.header = msg.header
            self.img_pub.publish(annotated)

    def log_stats(self):
        s = self.stats
        elapsed = time.monotonic() - s['since']
        if s['frames']:
            self.get_logger().info(
                f"{s['frames'] / elapsed:.1f} frames/s, {1000 * s['infer_s'] / s['frames']:.1f} ms "
                f"per frame, person seen in {100 * s['with_person'] / s['frames']:.0f}% of frames")
        else:
            self.get_logger().info('No camera images yet')
        self.stats = {'frames': 0, 'with_person': 0, 'infer_s': 0.0, 'since': time.monotonic()}


def main():
    rclpy.init()
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
