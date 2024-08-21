import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
from datetime import datetime

class ZEDCameraNode(Node):
    def __init__(self):
        super().__init__('zed_camera_node')
        self.publisher_ = self.create_publisher(Image, 'object_info', 10)
        self.bridge = CvBridge()
        self.create_timer(1.0, self.timer_callback)
        self.cap = cv2.VideoCapture(0)  # ZED2i 카메라 인덱스 또는 URL

        # 폴더가 없으면 생성
        if not os.path.exists('object_data'):
            os.makedirs('object_data')

        self.file_count = 0

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # 객체 인식 (여기서는 예를 들어 빈 리스트를 사용)
            detected_objects = self.detect_objects(frame)

            # 결과 화면 출력
            cv2.imshow('ZED2i Camera', frame)
            cv2.waitKey(1)

            # 이미지 메시지로 변환하여 퍼블리시
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(img_msg)

            # 객체 정보 저장
            self.save_object_info(detected_objects)

    def detect_objects(self, frame):
        # 여기에 실제 객체 인식 로직을 추가합니다
        # 현재는 빈 리스트를 반환합니다
        return []

    def save_object_info(self, detected_objects):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'object_data/object_info_{timestamp}_{self.file_count}.txt'
        self.file_count += 1

        with open(filename, 'w') as file:
            for obj in detected_objects:
                file.write(f'{obj}\n')  # 객체 정보 작성

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZEDCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
