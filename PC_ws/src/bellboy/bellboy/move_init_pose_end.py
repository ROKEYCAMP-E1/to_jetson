import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from rclpy.qos import QoSProfile
import math


class MoveInit(Node):
    def __init__(self):
        super().__init__('move_init')

        # 현재 위치를 저장
        self.current_pose = None
        self.initial_position = (0.1007787823513794, -0.01276879961445528)
        self.triggered = False  # 플래그 변수
        self.timer = None
        self.retry_timer = None

        # Action client for NavigateToPose
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Service terminate subscriber
        qos_profile = QoSProfile(depth=10)
        self.service_terminate_subscriber = self.create_subscription(
            String,
            '/ServiceTerminate',
            self.terminate_callback,
            qos_profile
        )

    def terminate_callback(self, msg):
        """
        Callback for service termination message.
        """
        if not self.triggered:  # 중복 실행 방지
            self.triggered = True
            self.get_logger().info(f"Service termination message received: {msg.data}")

            # 10초 대기를 Timer로 처리
            self.get_logger().info("Waiting for 10 seconds before getting current position...")
            self.timer = self.create_timer(10.0, self.start_amcl_subscription)

    def start_amcl_subscription(self):
        """
        Start AMCL subscription after the 10-second wait.
        """
        self.timer.cancel()  # 타이머 중단
        self.amcl_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )
        self.get_logger().info("Subscribed to AMCL pose to get current position.")

    def amcl_pose_callback(self, msg):
        """
        Callback for AMCL pose.
        """
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(
            f"AMCL position received: x={self.current_pose[0]}, y={self.current_pose[1]}"
        )

        # 구독 중단
        self.destroy_subscription(self.amcl_pose_subscriber)

        # 초기 위치로 이동 요청
        self.send_init_pose()

    def send_init_pose(self):
        """
        Send the robot to the initial position.
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # 초기 위치 설정
        goal_msg.pose.pose.position.x = self.initial_position[0]
        goal_msg.pose.pose.position.y = self.initial_position[1]
        goal_msg.pose.pose.position.z = 0.0

        # 방향 설정
        goal_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.get_logger().info('Waiting for action server...')
        if not self.action_client.wait_for_server(timeout_sec=10.0):  # 액션 서버 준비 대기
            self.get_logger().error('Action server not available. Exiting...')
            return

        self.get_logger().info('Sending goal to initial position...')
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        # 30초 타이머 설정
        self.retry_timer = self.create_timer(30.0, self.check_position_and_retry)

    def feedback_callback(self, feedback_msg):
        """
        Feedback callback for NavigateToPose action.
        """
        self.get_logger().info(f'Feedback received: {feedback_msg.feedback.current_pose.pose}')

    def goal_response_callback(self, future):
        """
        Callback for goal response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return
        self.get_logger().info('Goal accepted.')

        # 이동 완료 결과 대기
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        """
        Callback for action result.
        """
        result = future.result()
        if result:
            self.get_logger().info(f"Action result received: {result.status}")
        else:
            self.get_logger().warn("No result received from action server.")

    def check_position_and_retry(self):
        """
        Check if the robot has reached the initial position. If not, retry.
        """
        if self.current_pose is None:
            self.get_logger().warn("Current position is not available. Retrying...")
            self.start_amcl_subscription()
            return

        current_x, current_y = self.current_pose
        init_x, init_y = self.initial_position

        # 두 점 사이의 거리 계산
        distance = math.sqrt((current_x - init_x) ** 2 + (current_y - init_y) ** 2)

        self.get_logger().info(f"Distance to initial position: {distance:.2f} meters")

        if distance > 0.05:  
            self.get_logger().warn("Robot has not reached the initial position. Retrying...")
            self.send_init_pose()
        else:
            self.get_logger().info("Robot successfully reached the initial position.")
            self.retry_timer.cancel()


def main():
    rclpy.init()
    node = MoveInit()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
