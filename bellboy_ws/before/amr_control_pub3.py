import rclpy  # ROS 2 Python 라이브러리
from rclpy.node import Node
from std_msgs.msg import String  # ROS 2 메시지 타입
import cv2  # OpenCV 라이브러리
from ultralytics import YOLO  # YOLOv8 라이브러리

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')  # ROS 2 노드 이름 설정
        self.model = YOLO('AMR_epoch90.pt')  # YOLOv8 모델 로드 (가벼운 YOLOv8n 사용)
        self.cap = cv2.VideoCapture('/dev/video0')  # 카메라 초기화
        self.output_path = "output.jpg"  # 결과 이미지 저장 경로

        # 카메라 설정
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

        # YOLOv8 모델로 객체 탐지 및 추적 수행
        results = self.model.track(source=frame, persist=True)

        # 탐지 및 추적 결과가 있을 경우
        if len(results) > 0:
            frame_with_boxes = results[0].plot()  # 탐지 결과 시각화
            track_ids = results[0].boxes.id  # 트래킹된 객체 ID

            # 트래킹된 객체 ID와 상태 로그 출력 및 ROS 2 퍼블리싱
            for i, track_id in enumerate(track_ids):
                if track_id is not None:
                    feedback_msg = String()
                    feedback_msg.data = f"Object ID: {track_id} is being tracked."
                    self.feedback_publisher.publish(feedback_msg)
                    self.get_logger().info(f"Published feedback: {feedback_msg.data}")

            # 결과 이미지 저장
            cv2.imwrite(self.output_path, frame_with_boxes)
            self.get_logger().info(f"Frame with tracking saved to {self.output_path}.")
        else:
            self.get_logger().info("No detections in this frame.")

    def cleanup(self):
        self.cap.release()
        self.get_logger().info("Camera released.")


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    camera_processor = CameraProcessor()

    try:
        while rclpy.ok():
            camera_processor.process_frame()
    except KeyboardInterrupt:
        camera_processor.cleanup()
        camera_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
