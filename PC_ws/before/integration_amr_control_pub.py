import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
import math, sys
import cv2
from ultralytics import YOLO


class MoveGoal(Node):
    def __init__(self):
        super().__init__('move_goal')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None

        # 초기 위치 퍼블리셔
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # 탐지 결과를 받기 위한 구독자
        self.feedback_subscriber = self.create_subscription(
            String,
            '/feedback_topic',
            self.feedback_callback,
            10
        )

        # 탐지 상태 플래그
        self.detection_flag = False

    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self.euler_to_quaternion(0, 0, yaw)
        msg.pose.covariance = [0.25] * 36  # Covariance 기본값
        self.initial_pose_publisher.publish(msg)
        self.get_logger().info(f"Initial pose published: x={x}, y={y}, yaw={yaw}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        x = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        y = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        z = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        w = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=x, y=y, z=z, w=w)

    def send_goal(self, x, y, yaw):
        if self.detection_flag:  # 탐지 상태일 경우 목표 전송 중단
            self.get_logger().info('Detection in progress. Goal not sent.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = self.euler_to_quaternion(0, 0, yaw)
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Sending goal...')
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.goal_feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return
        self.get_logger().info('Goal accepted.')
        self._goal_handle = goal_handle

    def goal_feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Action Feedback: {feedback_msg.feedback}')

    def feedback_callback(self, msg):
        self.get_logger().info(f"Detection feedback received: {msg.data}")
        self.detection_flag = True  # 탐지 상태 활성화
        if self._goal_handle:  # 현재 진행 중인 Goal 취소
            self.cancel_goal()

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Canceling goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('No active goal to cancel.')

    def cancel_done_callback(self, future):
        self.get_logger().info('Goal canceled successfully.')

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.model = YOLO('AMR_epoch90.pt')  # YOLO 모델 로드
        self.cap = cv2.VideoCapture('/dev/video0')
        self.feedback_publisher = self.create_publisher(String, '/feedback_topic', 10)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame.")
            return
        results = self.model(frame)
        if len(results[0].boxes) > 0:
            feedback_msg = String()
            feedback_msg.data = "Object detected!"
            self.feedback_publisher.publish(feedback_msg)

    def cleanup(self):
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    move_goal = MoveGoal()
    camera_processor = CameraProcessor()

    def run_nodes():
        try:
            while rclpy.ok():
                camera_processor.process_frame()
                rclpy.spin_once(move_goal, timeout_sec=0.1)
        except KeyboardInterrupt:
            camera_processor.cleanup()
            move_goal.destroy_node()
            camera_processor.destroy_node()
            rclpy.shutdown()

    run_nodes()


if __name__ == '__main__':
    main()
