#!/usr/bin/env python3
"""Run Ultralytics YOLO inference on a ROS image stream."""

from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO

from yolov8_msgs.msg import InferenceResult, Yolov8Inference


class YoloCameraSubscriber(Node):
    """Publish typed detections and an annotated image for each camera frame."""

    def __init__(self) -> None:
        """Load the configured detector and initialize ROS interfaces."""
        super().__init__("yolov8_detector")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("image_topic", "/camera_sensor/image_raw")
        self.declare_parameter("inference_topic", "/yolov8/inference")
        self.declare_parameter("annotated_topic", "/yolov8/annotated_image")
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("device", "")

        model_path = str(Path(self.get_parameter("model_path").value).expanduser())
        self.confidence = float(self.get_parameter("confidence").value)
        self.device = str(self.get_parameter("device").value).strip() or None
        self.bridge = CvBridge()
        self.model = YOLO(model_path)

        self.image_subscription = self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.camera_callback,
            qos_profile_sensor_data,
        )
        self.inference_publisher = self.create_publisher(
            Yolov8Inference,
            self.get_parameter("inference_topic").value,
            10,
        )
        self.annotated_publisher = self.create_publisher(
            Image,
            self.get_parameter("annotated_topic").value,
            10,
        )
        self.get_logger().info(f"Loaded YOLO model: {model_path}")

    def camera_callback(self, message: Image) -> None:
        """Run inference on one camera frame and publish structured results."""
        try:
            image = self.bridge.imgmsg_to_cv2(message, "bgr8")
            results = self.model.predict(
                source=image,
                conf=self.confidence,
                device=self.device,
                verbose=False,
            )
            inference_message = Yolov8Inference()
            inference_message.header = message.header

            for result in results:
                for box in result.boxes:
                    x_min, y_min, x_max, y_max = box.xyxy[0].cpu().tolist()
                    detection = InferenceResult()
                    class_index = int(box.cls[0].item())
                    detection.class_name = str(self.model.names[class_index])
                    detection.confidence = float(box.conf[0].item())
                    detection.left = int(round(x_min))
                    detection.top = int(round(y_min))
                    detection.right = int(round(x_max))
                    detection.bottom = int(round(y_max))
                    inference_message.yolov8_inference.append(detection)

            annotated_image = results[0].plot()
            annotated_message = self.bridge.cv2_to_imgmsg(
                annotated_image, encoding="bgr8"
            )
            annotated_message.header = message.header
            self.annotated_publisher.publish(annotated_message)
            self.inference_publisher.publish(inference_message)
        except Exception as error:
            self.get_logger().error(f"YOLO inference failed: {error}")


def main(args: list[str] | None = None) -> None:
    """Run the YOLO inference node."""
    rclpy.init(args=args)
    node = YoloCameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
