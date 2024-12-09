import rclpy  # ROS 2 Python 라이브러리
from rclpy.node import Node
from sensor_msgs.msg import Image  # ROS 2 이미지 메시지 타입
from cv_bridge import CvBridge  # OpenCV와 ROS 메시지 변환 도구
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import cv2  # OpenCV 라이브러리


class RawImagePublisher(Node):
    def __init__(self): 
        super().__init__('raw_image_publisher')  # 노드 이름 설정

        # QoS 설정: 속도 위주 (BEST_EFFORT, VOLATILE)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 데이터 손실 가능
            durability=DurabilityPolicy.VOLATILE,      # 연결 중에만 데이터 유지
            depth=10                                   # 최근 10개의 메시지만 유지
        )

        # 이미지 퍼블리셔 생성
        self.publisher_ = self.create_publisher(Image, 'amr_view', qos_profile)
        self.get_logger().info('Raw image publisher created.')

        # OpenCV와 ROS 메시지 변환 도구
        self.bridge = CvBridge()

        # 타이머 생성: 30FPS로 퍼블리싱
        self.timer = self.create_timer(1 / 30.0, self.publish_image)
        self.get_logger().info('Timer set to 30 FPS.')

        # 카메라 초기화 (/dev/video0)
        self.cap = cv2.VideoCapture("/dev/video0")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # 해상도 설정 (가로)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # 해상도 설정 (세로)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera.')
            raise RuntimeError('Camera not found.')

    def publish_image(self):
        ret, frame = self.cap.read()  # 프레임 읽기
        if not ret:
            self.get_logger().error('Failed to capture image from camera.')
            return

        # OpenCV 이미지를 ROS 메시지로 변환
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        # 메시지 퍼블리싱
        self.publisher_.publish(image_msg)
        self.get_logger().info('Published image to amr_view.')

    def cleanup(self):
        self.cap.release()  # 카메라 자원 해제
        self.get_logger().info('Camera released.')


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = RawImagePublisher()

    try:
        rclpy.spin(node)  # 노드 실행
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.cleanup()  # 리소스 정리
        node.destroy_node()  # 노드 제거
        rclpy.shutdown()  # ROS 2 종료


if __name__ == '__main__':
    main()
