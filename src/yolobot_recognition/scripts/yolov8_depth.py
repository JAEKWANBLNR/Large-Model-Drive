#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Float32
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

from cv_bridge import CvBridge
from yolov8_msgs.msg import Yolov8Inference, InferenceResult

class DepthExtractor(Node):
    def __init__(self):
        super().__init__('depth_extractor')

        #Yolov8 추론 결를 구독하기 위한 구독자 생성 
        # /Yolov8_inference 토픽에서 Yolov8inference 메시지 구독
        self.subscription_yolo = self.create_subscription(
            Yolov8Inference, 
            '/Yolov8_Inference', 
            self.callback_yolo, 
            0)
        self.get_logger().info('Subscribed to /Yolov8_Inference topic.')

        #Depth 카메라에서 이미지를 구독하는 구독자 
        self.subscription_depth = self.create_subscription(
            PointCloud2, '/camera_sensor/points',
            self.callback_depth, 
            0)
        
        self.get_logger().info('Subscribed to /camera_sensor/depth/image_raw topic.')

        #'/depth_extractor' 토픽에 데이터를 퍼블리시할 퍼블리셔 생성 
        self.publisher_ = self.create_publisher(Float32, '/depth_extractor', 0)

        #인식된 사람의 바운딩 박스 정보를 저장할 리스트 
        self.bboxes =[]

        #Depth 이미지를 저장할 변수를 초기화 
        self.point_cloud = None
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.latest_avg_distance = None
        self.last_yolo_timestamp = None
        self.last_depth_timestep = None
        
        self.fx = 554.256  # Focal length in x
        self.fy = 554.256  # Focal length in y
        self.cx = 320.0  # Principal point in x
        self.cy = 240.0  # Principal point in y

    #Yolov8 추론결과 처리 콜백함수 
    def callback_yolo(self, msg: Yolov8Inference):
        
        # self.get_logger().info('Received message on /Yolov8_Inference topic.')

        #추론 결과에서 'person' 클래스를 가진 바운딩 박스를 필터링하여 bboxes리스트에 저장.
        self.bboxes = [
            bbox for bbox in msg.yolov8_inference if bbox.class_name == 'person'
        ]
        self.get_logger().info(f'Received {len(self.bboxes)} bounding boxes.')

    def callback_depth(self, msg: PointCloud2):
        # self.get_logger().info('Received message on /camera/depth/image_raw topic.')

        # Image 메시지 데이터를 numpy배열로 변환하여 self.depth_image 에 저장. 
        self.point_cloud = msg
        self.process_depth()

    #바운딩 박스 내의 depth 이미지 처리
    def process_depth(self):
        if not self.bboxes:
            self.get_logger().info('No bounding boxes to process.')
            return

        if self.point_cloud is None:
            self.get_logger().info('No point cloud data to process.')
            return

        distances = []
        points = pc2.read_points(self.point_cloud, field_names=("x","y","z"), skip_nans=True)
        points_list = list(points)
        self.get_logger().info(f'Point cloud contains {len(points_list)} points.')

        #각 바운딩 박스에 대해 반복 
        for bbox in self.bboxes:
            # 바운딩 박스의 좌상단, 우하단 좌표를 정수형으로 변환
            x_min, y_min = int(bbox.left), int(bbox.top)
            x_max, y_max = int(bbox.right), int(bbox.bottom)
            self.get_logger().info(f'Processing bbox with coordinates: x_min={x_min}, y_min={y_min}, x_max={x_max}, y_max={y_max}')

            bbox_points = [
                point for point in points_list
                if self.is_point_in_bbox(point, x_min, y_min, x_max, y_max)
            ]
            self.get_logger().info(f'Found {len(bbox_points)} points in the bounding box.')

            for x,y,z in bbox_points:
                self.get_logger().info(f"Point (x={x}, y={y}. z={z})")
                if z>0:
                    distances.append((x**2+y**2+z**2)**5)
        if distances:
            self.latest_avg_distance = np.mean(distances)
            self.get_logger().info(f"Avg distance : {self.latest_avg_distance}")
        else:
            self.latest_avg_distance = None
            self.get_logger().info('No valid depth data within bounding boxes')


    def timer_callback(self):
        self.get_logger().info('Timer callback triggered.')

        if self.latest_avg_distance is not None:
            distance_msg = Float32()
            distance_msg.data = self.latest_avg_distance
            self.publisher_.publish(distance_msg)
            self.get_logger().info(f"Pub avg distance:{self.latest_avg_distance}")
        else:
            self.get_logger().info("No valid depth data to publish")
    
    
    def is_point_in_bbox(self, point, x_min, y_min, x_max, y_max):
        """point cloud의 좌표가 bbox안에 있는지 확인 """
        x,y,z = point

        u = int((x * self.fx) / z + self.cx)
        v = int((y * self.fy) / z + self.cy)

        return x_min <= u <= x_max and y_min <= v <= y_max
    

def main(args=None):
        rclpy.init(args=args)
        depth_extractor = DepthExtractor()
        rclpy.spin(depth_extractor)
        depth_extractor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()