import rclpy  # ROS 2 Python 라이브러리
from rclpy.node import Node
from sensor_msgs.msg import Image  # ROS 2 이미지 메시지 타입
from cv_bridge import CvBridge  # ROS 이미지 변환 라이브러리
import cv2  # OpenCV 라이브러리
from ultralytics import YOLO  # YOLOv8 라이브러리


class AMRViewProcessor(Node):
    def __init__(self):
        super().__init__('amr_view_processor')  # ROS 2 노드 이름 설정
        self.bridge = CvBridge()  # ROS 이미지를 OpenCV 이미지로 변환
        self.model = YOLO('rccarV6.pt')  # YOLOv8 모델 로드
        self.subscription = self.create_subscription(
            Image,
            'amr_view',  # 구독할 토픽 이름
            self.listener_callback,
            10
        )
        self.output_path = "amr_view_image.jpg"  # 저장할 이미지 경로

    def listener_callback(self, msg):
        # ROS Image 메시지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLOv8 모델로 객체 탐지 수행
        results = self.model.track(source=frame, persist=True, conf=0.9)

        # 결과가 있는지 확인
        if len(results) > 0 and results[0].boxes is not None:
            track_ids = results[0].boxes.id.cpu().numpy() if results[0].boxes.id is not None else []
            boxes = results[0].boxes.xyxy.cpu().numpy()  # 바운딩 박스 가져오기

            # 탐지된 객체 시각화
            frame_with_boxes = results[0].plot()

            # 각 바운딩 박스와 ID 표시
            for box, track_id in zip(boxes, track_ids):
                xmin, ymin, xmax, ymax = map(int, box)
                box_center_x = int((xmin + xmax) / 2)
                box_center_y = int((ymin + ymax) / 2)
                cv2.circle(frame_with_boxes, (box_center_x, box_center_y), 5, (0, 0, 255), -1)

            # 결과 이미지 저장
            cv2.imwrite(self.output_path, frame_with_boxes)
            self.get_logger().info(f"Image with detections saved as {self.output_path}.")
        else:
            self.get_logger().info("No detections in this frame.")


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    amr_view_processor = AMRViewProcessor()

    try:
        rclpy.spin(amr_view_processor)  # ROS 2 노드 실행
    except KeyboardInterrupt:
        amr_view_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
