import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO  # YOLOv8 사용

class YOLOVideoPublisher(Node):
    def __init__(self, model_path):
        super().__init__('yolo_video_publisher')
        self.publisher_ = self.create_publisher(Image, 'amr_video', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 FPS
        self.cap = cv2.VideoCapture(0)  # video0 디바이스 열기

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open video device")
            raise RuntimeError("Cannot open video device")

        self.get_logger().info("Video device opened successfully")

        # YOLO 모델 로드
        self.model = YOLO(model_path)
        self.get_logger().info("YOLO model loaded successfully")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return

        # YOLO 모델로 객체 탐지
        results = self.model(frame)
        detections = results[0].boxes.data.cpu().numpy()

        # 탐지 결과를 이미지에 그리기
        for detection in detections:
            x1, y1, x2, y2, score, class_id = detection
            label = f"{self.model.names[int(class_id)]}: {score:.2f}"
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, label, (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # OpenCV 이미지를 ROS 2 메시지로 변환 후 퍼블리싱
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)
        self.get_logger().info("Detection image published")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    model_path = "AMR_epoch90.pt"  # YOLO 모델 경로 설정
    #model_path = "/home/kimdahun/rokey1_E1_ws/AMR_epoch90.pt"  # YOLO 모델 경로 설정
    yolo_video_publisher = YOLOVideoPublisher(model_path)

    try:
        rclpy.spin(yolo_video_publisher)
    except KeyboardInterrupt:
        pass

    yolo_video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
