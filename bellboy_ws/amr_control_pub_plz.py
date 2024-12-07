import rclpy  # ROS 2 Python 라이브러리
from rclpy.node import Node
from std_msgs.msg import String  # ROS 2 메시지 타입
from sensor_msgs.msg import Image  # ROS 2 이미지 메시지 타입
from cv_bridge import CvBridge  # OpenCV와 ROS 메시지 간 변환 도구
import cv2  # OpenCV 라이브러리
from ultralytics import YOLO  # YOLOv8 라이브러리


class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')  # ROS 2 노드 이름 설정
        self.model = YOLO('rccarV6.pt')  # YOLOv8 모델 로드
        self.cap = cv2.VideoCapture('/dev/video0')  # /dev/video2 카메라 사용
        self.output_path = "output.jpg"  # 저장할 이미지 경로

        # 카메라 설정
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # ROS 2 퍼블리셔 생성
        self.feedback_publisher = self.create_publisher(String, '/feedback_topic', 10)
        self.image_publisher = self.create_publisher(Image, 'amr_view', 10)  # amr_view 퍼블리셔 생성

        # OpenCV와 ROS 메시지 간 변환 도구
        self.bridge = CvBridge()

    def process_frame(self):
        # 카메라에서 프레임 읽기
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame from camera.")
            return

        # YOLOv8 모델로 객체 탐지 수행 (트래킹 없음, conf=0.9 적용)
        results = self.model.predict(source=frame, conf=0.9)

        # 결과가 있는지 확인
        if len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()  # 바운딩 박스 가져오기
            confidences = results[0].boxes.conf.cpu().numpy()  # 신뢰도 가져오기
            classes = results[0].boxes.cls.cpu().numpy()  # 클래스 가져오기
        else:
            self.get_logger().info("No detections in this frame.")
            return

        # 탐지된 객체 처리
        frame_with_boxes = results[0].plot()
        image_height, image_width, _ = frame_with_boxes.shape
        center_x = image_width // 2
        center_y = image_height // 2

        # 그린존 영역 추가
        cv2.line(frame_with_boxes, (center_x - 150, 0), (center_x - 30, image_height), (0, 255, 0), 5)
        cv2.line(frame_with_boxes, (center_x + 150, 0), (center_x + 30, image_height), (0, 255, 0), 5)
        cv2.line(frame_with_boxes, (0, center_y + 200), (image_width, center_y + 200), (0, 255, 0), 5)

        # 각 바운딩 박스 처리
        for i, box in enumerate(boxes):
            xmin, ymin, xmax, ymax = map(int, box)
            box_center_x = int((xmin + xmax) / 2)
            box_center_y = int((ymin + ymax) / 2)

            # 박스 중심 표시
            cv2.circle(frame_with_boxes, (box_center_x, box_center_y), 5, (0, 0, 255), -1)

            # Green Zone 판단 및 피드백
            feedback_msg = String()
            if box_center_y >= center_y + 200:
                feedback_msg.data = f"Object {i}: Move robot down!"
            elif box_center_x > center_x + 150:
                feedback_msg.data = f"Object {i}: Move amr right!"
            elif box_center_x < center_x - 150:
                feedback_msg.data = f"Object {i}: Move amr left!"
            elif box_center_y < center_y + 200:
                feedback_msg.data = f"Object {i}: Move robot up!"

            # 퍼블리시 피드백 메시지
            self.feedback_publisher.publish(feedback_msg)
            self.get_logger().info(f"\nPublished feedback: {feedback_msg.data}")

        # amr_view 토픽으로 이미지 퍼블리싱
        ros_image = self.bridge.cv2_to_imgmsg(frame_with_boxes, encoding="bgr8")
        self.image_publisher.publish(ros_image)
        self.get_logger().info("Published a frame to 'amr_view'.")

        # 탐지된 이미지 저장
        cv2.imwrite(self.output_path, frame_with_boxes)
        self.get_logger().info(f"Frame with detections saved to {self.output_path}.")

    def cleanup(self):
        self.cap.release()
        self.get_logger().info("Camera released.")


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    camera_processor = CameraProcessor()

    try:
        while rclpy.ok():  # ROS 2 루프
            rclpy.spin_once(camera_processor, timeout_sec=0.1)  # ROS 이벤트 처리
            camera_processor.process_frame()
    except KeyboardInterrupt:
        camera_processor.cleanup()
        camera_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
