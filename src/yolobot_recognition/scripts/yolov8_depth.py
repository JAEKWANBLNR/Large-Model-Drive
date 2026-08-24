#!/usr/bin/env python3
"""Estimate object distance from organized PointCloud2 samples."""

import math
import statistics

import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32

from yolov8_msgs.msg import Yolov8Inference


class DepthExtractor(Node):
    """Publish the median 3D distance for recent target-class detections."""

    def __init__(self) -> None:
        """Configure synchronized detections and point-cloud sampling."""
        super().__init__("depth_extractor")
        self.declare_parameter("inference_topic", "/yolov8/inference")
        self.declare_parameter("point_cloud_topic", "/camera_sensor/points")
        self.declare_parameter("distance_topic", "/depth_extractor")
        self.declare_parameter("target_class", "person")
        self.declare_parameter("minimum_confidence", 0.25)
        self.declare_parameter("sample_grid_size", 7)
        self.declare_parameter("detection_timeout", 1.0)

        self.target_class = str(self.get_parameter("target_class").value)
        self.minimum_confidence = float(self.get_parameter("minimum_confidence").value)
        self.sample_grid_size = max(
            1, int(self.get_parameter("sample_grid_size").value)
        )
        self.detection_timeout = float(self.get_parameter("detection_timeout").value)
        self.detections = []
        self.last_detection_time = None

        self.inference_subscription = self.create_subscription(
            Yolov8Inference,
            self.get_parameter("inference_topic").value,
            self.inference_callback,
            10,
        )
        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter("point_cloud_topic").value,
            self.point_cloud_callback,
            qos_profile_sensor_data,
        )
        self.distance_publisher = self.create_publisher(
            Float32,
            self.get_parameter("distance_topic").value,
            10,
        )

    def inference_callback(self, message: Yolov8Inference) -> None:
        """Cache the newest target detection for the next point cloud."""
        self.detections = [
            detection
            for detection in message.yolov8_inference
            if detection.class_name == self.target_class
            and detection.confidence >= self.minimum_confidence
        ]
        self.last_detection_time = self.get_clock().now()

    def point_cloud_callback(self, cloud: PointCloud2) -> None:
        """Estimate median Euclidean distance from samples inside the target box."""
        if not self.detections or self.last_detection_time is None:
            return
        age = (self.get_clock().now() - self.last_detection_time).nanoseconds
        if age / 1e9 > self.detection_timeout:
            return
        if cloud.height <= 1:
            self.get_logger().warning(
                "Point cloud is unorganized; pixel-aligned depth is unavailable."
            )
            return

        distances = []
        for detection in self.detections:
            uvs = self._sample_pixels(detection, cloud.width, cloud.height)
            points = pc2.read_points(
                cloud,
                field_names=("x", "y", "z"),
                skip_nans=True,
                uvs=uvs,
            )
            for point in points:
                x, y, z = (float(point[index]) for index in range(3))
                if z > 0.0:
                    distances.append(math.sqrt(x * x + y * y + z * z))

        if not distances:
            return
        output = Float32()
        output.data = float(statistics.median(distances))
        self.distance_publisher.publish(output)
        self.get_logger().debug(f"Published median distance: {output.data:.3f} m")

    def _sample_pixels(
        self, detection: object, width: int, height: int
    ) -> list[tuple[int, int]]:
        left = min(max(int(detection.left), 0), width - 1)
        right = min(max(int(detection.right), left), width - 1)
        top = min(max(int(detection.top), 0), height - 1)
        bottom = min(max(int(detection.bottom), top), height - 1)

        def samples(start: int, end: int) -> list[int]:
            if self.sample_grid_size == 1 or start == end:
                return [(start + end) // 2]
            step = (end - start) / (self.sample_grid_size - 1)
            return [
                int(round(start + index * step))
                for index in range(self.sample_grid_size)
            ]

        return [(x, y) for y in samples(top, bottom) for x in samples(left, right)]


def main(args: list[str] | None = None) -> None:
    """Run the YOLO depth-estimation node."""
    rclpy.init(args=args)
    node = DepthExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
