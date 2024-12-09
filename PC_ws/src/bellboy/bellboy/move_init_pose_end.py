import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from rclpy.qos import QoSProfile


class MoveInit(Node):
    def __init__(self):
        super().__init__('move_init')

        # 현재 위치를 저장
        self.current_pose = None
        self.triggered = False  # 플래그 변수
        self.timer = None

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
        if self.current_pose is None:  # 첫 번째 위치만 처리
            self.current_pose = msg  # 현재 위치 저장
            self.get_logger().info(
                f"AMCL position received: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}"
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

        # 초기 위치 설정 (예제 좌표)
        goal_msg.pose.pose.position.x = 0.1007787823513794
        goal_msg.pose.pose.position.y = -0.01276879961445528
        goal_msg.pose.pose.position.z = 0.0

        # 방향 설정
        goal_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()  # 액션 서버 준비 대기
        self.get_logger().info('Sending goal to initial position...')

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """
        Feedback callback for NavigateToPose action.
        """
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.current_pose.pose}')

    def goal_response_callback(self, future):
        """
        Callback for goal response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return
        self.get_logger().info('Goal accepted.')


def main():
    rclpy.init()
    node = MoveInit()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
