#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image #ROS2의 표준 이미지 메시지 타입 
from cv_bridge import CvBridge #ROS2 이미지 메시지와 OpenCV 이미지간의 변환을 도움 

from yolov8_msgs.msg import InferenceResult #메시지 타입 임포트 
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge() # bridge 인스턴스 생성, ROS이미지를 OpenCV이미지로 변환 

class Yolo_camera_subscriber(Node):

    def __init__(self):
        super().__init__('yolo_camera_subscriber') #노드 이름 'camera_subscriber' 

        #yolov8 모델 불러오기 
        self.model = YOLO('~/yolobot/src/yolobot_recognition/scripts/yolov8n.pt')

        #Yolov8Inference 메시지 객체 초기화, 추론 결과 담음 
        self.yolov8_inference = Yolov8Inference()
        
        # 'Image' 타입의 메시지를 camera_sensor/image_raw topic에서 구독. 
        self.subscription = self.create_subscription(
            Image,
            'camera_sensor/image_raw',
            self.camera_callback,
            10)
        self.subscription 
        # /Yolo8_Inference 토픽에 메시지 pub 
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)

        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    def camera_callback(self, data): # data: /camera_sensor/image_raw 토픽에서 수신된 ROS 이미지 
        #  ROS image 메시지를 openCV 형식으로 전환, BGR 8비트 형식 
        img = bridge.imgmsg_to_cv2(data, "bgr8")

        # Load된 YOLOv8 모델에 변환된 image를 집어 넣고, result 객체에 할당. 
        results = self.model(img)

        # 메시지의 header에 담을 내용 
        self.yolov8_inference.header.frame_id = "inference" #메시지 헤더의 frame_id를 'inference' 로 설정 
        self.yolov8_inference.header.stamp = yolo_camera_subscriber.get_clock().now().to_msg() #현재시간을 메시지 헤더의 타임스탬프로 설정 

        for r in results: #모델 추론 결과를 반복 
            boxes = r.boxes #각 결과에서 검출된 bbox 를 추출 
            for box in boxes:
                self.inference_result = InferenceResult() #InferenceResult 메시지 초기화, 객체생성 
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls #검출된 객체의 클래스 index를 추출 
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.yolov8_inference.yolov8_inference.append(self.inference_result) #inference result 객체를 Yolov8Inferecne 메시지의 두번째 열,yolov8_inference 배열에 추가 

            #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")

        # 이미지 주석 및 결과 publish 
        annotated_frame = results[0].plot() # 첫번째 결과 이미지에 주석을 추가 (plot()은 bbox와 라벨을 그리는 역할을 함)
        img_msg = bridge.cv2_to_imgmsg(annotated_frame) #주석이 추가된 이미지를 ros이미지 메시지로 변환 

        self.img_pub.publish(img_msg) #/Infrence_Result 토픽에 ros이미지 메시지를 퍼블리시 
        self.yolov8_pub.publish(self.yolov8_inference) #메시지를 /Yolov8_inferecne 토픽에 pub 
        self.yolov8_inference.yolov8_inference.clear() # yolov8Inference 메시지 객체의 리스트를 초기화 

if __name__ == '__main__':
    rclpy.init(args=None)
    yolo_camera_subscriber = Yolo_camera_subscriber()
    rclpy.spin(yolo_camera_subscriber)
    rclpy.shutdown()
