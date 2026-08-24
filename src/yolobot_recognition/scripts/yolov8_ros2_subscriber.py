#!/usr/bin/env python3
"""Render received YOLO detections over the latest RGB frame."""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from yolov8_msgs.msg import Yolov8Inference


class YoloOverlay(Node):
    """Combine camera and detection subscriptions in one race-free node."""

    def __init__(self) -> None:
        """Initialize image, detection, and annotated-image ROS interfaces."""
        super().__init__("yolov8_overlay")
        self.declare_parameter("image_topic", "/camera_sensor/image_raw")
        self.declare_parameter("inference_topic", "/yolov8/inference")
        self.declare_parameter("overlay_topic", "/yolov8/overlay_image")
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_header = None
        self.image_subscription = self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.inference_subscription = self.create_subscription(
            Yolov8Inference,
            self.get_parameter("inference_topic").value,
            self.inference_callback,
            10,
        )
        self.overlay_publisher = self.create_publisher(
            Image,
            self.get_parameter("overlay_topic").value,
            10,
        )

    def image_callback(self, message: Image) -> None:
        """Cache the latest camera frame for annotation."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(message, "bgr8").copy()
            self.latest_header = message.header
        except Exception as error:
            self.get_logger().error(f"Image conversion failed: {error}")

    def inference_callback(self, message: Yolov8Inference) -> None:
        """Draw received detections on the latest frame and publish it."""
        if self.latest_image is None:
            self.get_logger().warning("Detection arrived before the first image.")
            return
        image = self.latest_image.copy()
        for detection in message.yolov8_inference:
            top_left = (detection.left, detection.top)
            bottom_right = (detection.right, detection.bottom)
            cv2.rectangle(image, top_left, bottom_right, (255, 255, 0), 2)
            label = f"{detection.class_name} {detection.confidence:.2f}"
            cv2.putText(
                image,
                label,
                (detection.left, max(15, detection.top - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
                1,
                cv2.LINE_AA,
            )
        output = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        output.header = message.header or self.latest_header
        self.overlay_publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    """Run the YOLO visualization node."""
    rclpy.init(args=args)
    node = YoloOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
