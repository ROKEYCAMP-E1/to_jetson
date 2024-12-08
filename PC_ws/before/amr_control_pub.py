import rclpy  # ROS 2 Python 라이브러리
from rclpy.node import Node
from std_msgs.msg import String  # ROS 2 메시지 타입
import cv2  # OpenCV 라이브러리
from ultralytics import YOLO  # YOLOv8 라이브러리
import time

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')  # ROS 2 노드 이름 설정
        self.model = YOLO('AMR_epoch90.pt')  # YOLOv8 모델 로드
        self.cap = cv2.VideoCapture('/dev/video0')  # /dev/video2 카메라 사용
        self.output_path = "output.jpg"  # 저장할 이미지 경로

        # 카메라 설정
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # ROS 2 퍼블리셔 생성
        self.feedback_publisher = self.create_publisher(String, '/feedback_topic', 10)

    def process_frame(self):
        # 카메라에서 프레임 읽기
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame from camera.")
            return
        #time.sleep(0.5)#0.5초 대기

        # YOLOv8 모델로 객체 탐지 수행
        results = self.model(frame)

        # 탐지된 이미지가 있을 경우
        if len(results[0].boxes) > 0:
            frame_with_boxes = results[0].plot()
            image_height, image_width, _ = frame_with_boxes.shape
            center_x = image_width // 2
            center_y = image_height // 2

            # 그린존 영역 추가
            cv2.line(frame_with_boxes, (center_x - 150, center_y + 100), (center_x - 150, image_height), (0, 255, 0), 5)
            cv2.line(frame_with_boxes, (center_x + 150, center_y + 100), (center_x + 150, image_height), (0, 255, 0), 5)
            cv2.line(frame_with_boxes, (center_x - 150, center_y + 100), (center_x + 150, center_y + 100), (0, 255, 0), 5)
            cv2.line(frame_with_boxes, (center_x - 150, image_height), (center_x + 150, image_height), (0, 255, 0), 5)

            # 바운딩 박스 정보 가져오기
            boxes = results[0].boxes.xyxy.cpu().numpy()
            for i, box in enumerate(boxes):
                xmin, ymin, xmax, ymax = map(int, box)
                box_center_x = int((xmin + xmax) / 2)
                box_center_y = int((ymin + ymax) / 2)

                # 박스 중심 표시
                cv2.circle(frame_with_boxes, (box_center_x, box_center_y), 5, (0, 0, 255), -1)

                # Green Zone 판단 및 피드백
                feedback_msg = String()
                if center_x - 150 <= box_center_x <= center_x + 150 and center_y + 100 <= box_center_y:
                    feedback_msg.data = "Catch box!"
                elif box_center_x > center_x + 150:
                    feedback_msg.data = "Move arm right!"
                elif box_center_x < center_x - 150:
                    feedback_msg.data = "Move arm left!"
                elif box_center_y < center_y + 100:
                    feedback_msg.data = "Move robot up!"

                # 퍼블리시 피드백 메시지
                self.feedback_publisher.publish(feedback_msg)
                self.get_logger().info(f"Published feedback: {feedback_msg.data}")

            # 탐지된 이미지 저장
            cv2.imwrite(self.output_path, frame_with_boxes)
            self.get_logger().info(f"Frame with detections saved to {self.output_path}.")
        else:
            self.get_logger().info("No detections in this frame.")

    def cleanup(self):
        self.cap.release()
        self.get_logger().info("Camera released.")


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    camera_processor = CameraProcessor()

    try:
        while rclpy.ok():  # ROS 2 루프
            camera_processor.process_frame()
    except KeyboardInterrupt:
        camera_processor.cleanup()
        camera_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
