import rclpy  # ROS 2 Python 라이브러리
from rclpy.node import Node
from sensor_msgs.msg import Image  # ROS 2 이미지 메시지 타입
from std_msgs.msg import String, Bool  # ROS 2 문자열 메시지 타입
from cv_bridge import CvBridge  # OpenCV와 ROS 메시지 변환 도구
import cv2  # OpenCV 라이브러리
from ultralytics import YOLO  # YOLOv8 라이브러리
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')  # 노드 이름 설정
        self.tracking_start = 0
        self.detectperson = False  # 감지 상태 초기화
        self.switch_detect = True #detect 연락을 보내면 False
        self.customer_decision_received = False
        self.switch_decision = True #decision 연락을 받으면 False

        # YOLO 모델 초기화
        self.model = YOLO('rccarV6.pt')  # YOLOv8 모델 로드

        # QoS 설정: 속도 위주
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 데이터 손실 가능
            durability=DurabilityPolicy.VOLATILE,      # 연결 중에만 데이터 유지
            depth=10                                   # 최근 10개의 메시지만 유지
        )

        # 이미지 토픽 구독 설정
        self.subscription = self.create_subscription(
            Image,
            'amr_view',  # 구독할 토픽 이름
            self.listener_callback,
            qos_profile
        )

        # CustomerDecision 토픽 구독 설정
        self.customer_decision_subscription = self.create_subscription(
            Bool,
            'CustomerDecision',  # 구독할 토픽 이름
            self.customer_decision_callback,  # 콜백 함수
            10
        )

        # 피드백 퍼블리셔 설정
        self.feedback_publisher = self.create_publisher(String, '/feedback_topic', 10)

        # DetectPerson 퍼블리셔 설정
        self.detectperson_publisher = self.create_publisher(Bool, '/DetectPerson', 10)

        # ServiceTerminate 퍼블리셔 설정
        self.service_terminate_publisher = self.create_publisher(String, '/ServiceTerminate', 10)

        self.bridge = CvBridge()  # CvBridge 초기화
        self.output_path = "tracking_image.jpg"  # 저장할 이미지 경로

        # 타이머 설정 (10초 주기)
        self.timer = self.create_timer(40.0, self.check_service_terminate)

        self.get_logger().info("ImageProcessor Node has been initialized and is subscribing to 'amr_view'.")

    def listener_callback(self, msg):
        # ROS 메시지를 OpenCV 이미지로 변환
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV: {e}")
            return

        # 이미지 해상도 조정 (1280x720)
        frame = cv2.resize(frame, (1280, 720))

        # YOLOv8 모델로 객체 탐지 및 트래킹 수행
        results = self.model.track(source=frame, conf=0.5, persist=True)

        # 결과가 있는지 확인
        if len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()  # 바운딩 박스 가져오기
            confidences = results[0].boxes.conf.cpu().numpy()  # 신뢰도 가져오기
            classes = results[0].boxes.cls.cpu().numpy()  # 클래스 가져오기

            # 객체 ID가 None인지 확인하고 처리
            if results[0].boxes.id is not None:
                ids = results[0].boxes.id.cpu().numpy()  # 객체 ID 가져오기
                if self.switch_detect:
                    self.detectperson = True
            else:
                ids = [-1] * len(boxes)  # ID가 없을 경우 기본값 설정
        else:
            self.get_logger().info("No detections in this frame.")
            return

        if self.detectperson and self.switch_detect:
            self.publish_detect_person(True)
            self.switch_detect = False#스위치를 꺼서 최초 1회만 보내도록 함

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

            # ID가 1인 객체만 처리. 이 기능을 사용하려면 아래 두 줄의 주석을 해제해야함
            # if ids[i] != 1:
            #     continue

            # 박스 중심 표시
            cv2.circle(frame_with_boxes, (box_center_x, box_center_y), 5, (0, 0, 255), -1)

            # Green Zone 판단 및 피드백
            feedback_msg = String()
            if box_center_y >= center_y + 200:
                feedback_msg.data = f"Object {i} (ID: 1): Move robot down!"
            elif box_center_x > center_x + 150:
                feedback_msg.data = f"Object {i} (ID: 1): Move amr right!"
            elif box_center_x < center_x - 150:
                feedback_msg.data = f"Object {i} (ID: 1): Move amr left!"
            elif box_center_y < center_y + 200:
                feedback_msg.data = f"Object {i} (ID: 1): Move robot up!"
            
            if (not self.switch_decision) and self.customer_decision_received:#false, true는 tracking 시작
                # 퍼블리시 피드백 메시지
                self.feedback_publisher.publish(feedback_msg)
                self.get_logger().info(f"\nPublished feedback: {feedback_msg.data}")

        # 탐지된 이미지 저장
        cv2.imwrite(self.output_path, frame_with_boxes)
        self.get_logger().info(f"Processed image saved to {self.output_path}.")

    def customer_decision_callback(self, msg):#USER
        if msg.data:  # 수신한 Bool 메시지의 data 필드가 True인지 확인
            self.get_logger().info("CustomerDecision message received: True")
            self.customer_decision_received = True
        else:
            self.get_logger().info("CustomerDecision message received: False")
            self.customer_decision_received = False
        self.switch_decision = False#decision 스위치 끔 = customerdecision 연락이 왔는가
        #false, false는 예외적인 연락
        #false, true는 tracking 시작
        #true, false는 연락안옴 10초지남일지도.


    def publish_detect_person(self, status):
        # Bool 메시지를 퍼블리시
        detect_msg = Bool()
        detect_msg.data = status
        self.detectperson_publisher.publish(detect_msg)
        self.get_logger().info(f"DetectPerson published: {status}")

    def check_service_terminate(self):
        # 10초 후 switch_decision이 True로 유지되었는지 확인
        if self.switch_decision:
            terminate_msg = String()
            terminate_msg.data = "USER"
            self.service_terminate_publisher.publish(terminate_msg)
            self.get_logger().info(f"Published ServiceTerminate message: {terminate_msg.data}")

    def cleanup(self):
        self.get_logger().info("Shutting down ImageProcessor Node.")


def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()

    try:
        rclpy.spin(image_processor)
    except KeyboardInterrupt:
        image_processor.get_logger().info('Shutting down...')
    finally:
        image_processor.cleanup()
        image_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
