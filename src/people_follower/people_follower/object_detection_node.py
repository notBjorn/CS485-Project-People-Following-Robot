
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('object_detection_node')

        # Configurable settings
        self.declare_parameter('model_path', 'yolo26n.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter(
            'image_topic',
            '/oakd/rgb/preview/image_raw'
        )

        model_path = self.get_parameter('model_path').value
        self.confidence = self.get_parameter('confidence').value
        image_topic = self.get_parameter('image_topic').value

        # Load YOLO model
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        # Subscribe to TurtleBot camera
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # Publish object detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/object_detection/detections',
            10
        )

        # Publish annotated camera image
        self.image_pub = self.create_publisher(
            Image,
            '/object_detection/annotated_image',
            10
        )

        self.get_logger().info(
            f'YOLO object detection started: {model_path}'
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )

            results = self.model.predict(
                source=frame,
                conf=self.confidence,
                verbose=False
            )

            detection_array = Detection2DArray()
            detection_array.header = msg.header

            for result in results:
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])

                    x1, y1, x2, y2 = box.xyxy[0].tolist()

                    detection = Detection2D()
                    detection.header = msg.header

                    detection.bbox.center.x = (x1 + x2) / 2
                    detection.bbox.center.y = (y1 + y2) / 2

                    detection.bbox.size_x = x2 - x1
                    detection.bbox.size_y = y2 - y1

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(
                        self.model.names[class_id]
                    )
                    hypothesis.hypothesis.score = confidence

                    detection.results.append(hypothesis)
                    detection_array.detections.append(detection)

                # Draw boxes and labels
                annotated = result.plot()

                annotated_msg = self.bridge.cv2_to_imgmsg(
                    annotated, encoding='bgr8'
                )
                annotated_msg.header = msg.header
                self.image_pub.publish(annotated_msg)

            self.detection_pub.publish(detection_array)

        except Exception as error:
            self.get_logger().error(
                f'Object detection error: {error}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
