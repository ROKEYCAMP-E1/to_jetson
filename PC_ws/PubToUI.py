import rclpy  # ROS 2 Python 라이브러리
from rclpy.node import Node
from sensor_msgs.msg import Image  # ROS 2 이미지 메시지 타입
from cv_bridge import CvBridge  # OpenCV와 ROS 메시지 변환 도구
import cv2  # OpenCV 라이브러리
from ultralytics import YOLO  # YOLOv8 라이브러리
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class ImageProcessorWithPublisher(Node):
    def __init__(self):
        super().__init__('image_processor_with_publisher')  # 노드 이름 설정
        self.bridge = CvBridge()  # CvBridge 초기화
        self.model = YOLO('rccarV6.pt')  # YOLO 모델 초기화
        self.output_path = "tracked_image.jpg"  # 디버깅용 저장 경로

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # 구독 설정
        self.subscription = self.create_subscription(
            Image,
            'amr_view',  # 입력 토픽
            self.listener_callback,
            qos_profile
        )

        # 발행 설정
        self.image_publisher = self.create_publisher(
            Image,
            'amr_video',  # 출력 토픽
            qos_profile
        )

        self.get_logger().info("ImageProcessorWithPublisher Node initialized and subscribing to 'amr_view'.")

    def listener_callback(self, msg):
        try:
            # ROS 메시지를 OpenCV 이미지로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV: {e}")
            return

        # 이미지 해상도 조정
        frame = cv2.resize(frame, (320, 180))

        # YOLOv8 추적 수행
        results = self.model.track(source=frame, conf=0.7, persist=True)

        # 결과 확인 및 바운딩 박스 그리기
        if len(results) > 0 and results[0].boxes is not None:
            frame_with_boxes = results[0].plot()  # 바운딩 박스가 그려진 이미지
        else:
            self.get_logger().info("No detections in this frame.")
            frame_with_boxes = frame

        # 이미지를 저장 (디버깅용, 필요 시 제거 가능)
        cv2.imwrite(self.output_path, frame_with_boxes)

        # OpenCV 이미지를 ROS 메시지로 변환하여 퍼블리시
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame_with_boxes, encoding='bgr8')
            self.image_publisher.publish(image_msg)
            self.get_logger().info("Tracked image published to 'amr_video'.")
        except Exception as e:
            self.get_logger().error(f"Failed to convert OpenCV image to ROS Image message: {e}")


def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessorWithPublisher()

    try:
        rclpy.spin(image_processor)
    except KeyboardInterrupt:
        image_processor.get_logger().info('Shutting down...')
    finally:
        image_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
