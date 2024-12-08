#move_goal4
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion,PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
import math, sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,String
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class MoveGoal(Node):
    def __init__(self):
        super().__init__('move_goal')
        
        # QoS 설정: 속도 위주
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 데이터 손실 가능
            durability=DurabilityPolicy.VOLATILE,      # 연결 중에만 데이터 유지
            depth=10                                   # 최근 10개의 메시지만 유지
        )
        self._goal_handle = None
        self.goal_sent = False  # Goal이 한 번만 전송되도록 관리
        self.goal_canceled = False  # Goal이 한 번만 취소되도록 관리
        self.timer = None  # 타이머 객체
        self.detect_person = False
        self.detect_luggage = False


        self.action_client = ActionClient(
            self, FollowWaypoints, 
            '/follow_waypoints'
        )
        
    
        # 초기 위치 퍼블리셔 추가
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.DetectLuggage_subscriber = self.create_subscription(
            Bool,  # 메시지 타입
            '/DetectLuggage',  # 토픽 이름
            self.detect_luggage_callback, # 콜백 함수
            qos_profile
        )

        self.DetectPerson_subscriber = self.create_subscription(
            Bool,  # 메시지 타입
            'DetectPerson',  # 토픽 이름
            self.detect_person_callback,  # 콜백 함수
            qos_profile  # QoS 설정
        )

        self.service_terminate_publisher = self.create_publisher(
            String, 
            '/service_terminate', 
            10)
        
    def detect_luggage_callback(self, msg):
        """DetectLuggage 콜백 함수: Luggage를 감지하면 액션 실행"""
        self.get_logger().info(f"Received DetectLuggage message: {msg.data}")
        
        if msg.data:  # 감지된 경우
            self.detect_luggage = True
            self.get_logger().info("Luggage detected. Starting action.")
            self.start_action_after_delay()  # 액션 실행


    def detect_person_callback(self, msg):
        # 메시지 내용 확인
        self.get_logger().info(f"Received detection message: {msg.data}")
        
        # 특정 조건 만족 시 goal 취소
        if msg.data==1 and not self.goal_canceled:  # 메시지 조건 및 중복 실행 방지
            self.get_logger().info("Condition met. Canceling goal...")
            self.cancel_goal()
            self.goal_canceled = True  # Goal 취소 플래그 설정

    def start_action_after_delay(self):
        """DetectLuggage 감지 후 10초 대기 후 동작"""
        self.get_logger().info("Starting action after delay.")
        self.send_goal()
        self.destroy_timer(self.timer)

    # 초기 위치를 퍼블리시
    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        # 위치 설정
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # 방향 설정 (쿼터니언)
        msg.pose.pose.orientation = self.euler_to_quaternion(0, 0, yaw)

        # Covariance 설정 (기본값)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
		]

        # 초기 위치 퍼블리시
        self.initial_pose_publisher.publish(msg)
        self.get_logger().info(f"Initial pose published: x={x}, y={y}, yaw={yaw}")

    # 오일러 각을 쿼터니언으로 변환
    def euler_to_quaternion(self, roll, pitch, yaw):
        x = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        y = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        z = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        w = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=x, y=y, z=z, w=w)

    # Nav2에 목표 좌표와 각 전달
    def send_goal(self):
        if self.goal_sent:
            self.get_logger().info('Goal already sent. Ignoring duplicate request.')
            return
        waypoints = []

        waypoint1 = PoseStamped()
        waypoint1.header.stamp.sec = 0
        waypoint1.header.stamp.nanosec = 0
        waypoint1.header.frame_id = "map"  # 프레임 ID를 설정 (예: "map")
        waypoint1.pose.position.x = 0.24466873107422488
        waypoint1.pose.position.y = -0.4085067988873908
        waypoint1.pose.position.z = 0.0

        waypoint1_yaw = 0.0
        waypoint1.pose.orientation = self.euler_to_quaternion(0, 0, waypoint1_yaw)

        waypoints.append(waypoint1)


        waypoint2 = PoseStamped()
        waypoint2.header.stamp.sec = 0
        waypoint2.header.stamp.nanosec = 0
        waypoint2.header.frame_id = "map"  # 프레임 ID를 설정 (예: "map")
        waypoint2.pose.position.x = -1.2978199371227226
        waypoint2.pose.position.y = -0.24750621448314233
        waypoint2.pose.position.z = 0.0

        waypoint2_yaw = 0.0
        waypoint2.pose.orientation = self.euler_to_quaternion(0, 0, waypoint2_yaw)

        waypoints.append(waypoint2)

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints


        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Sending goal...')

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_sent = True  # Goal 전송 플래그 설정


        self.timer = self.create_timer(30.0, self.check_person_after_timeout)
        self.get_logger().info("Timer for 30 seconds has been started.")

    def check_person_after_timeout(self):
        if not self.detect_person:  # DetectPerson 값이 False이면 서비스 종료
            self.get_logger().info('30 seconds passed, no person detected. Terminating service.')

            # 서비스 종료 메시지 발행
            terminate_msg = String()
            terminate_msg.data = "AMR"
            self.service_terminate_publisher.publish(terminate_msg)
            
            if self._goal_handle is not None:
                self.cancel_goal()
                
            # 타이머 삭제
            self.destroy_timer(self.timer)



    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.current_waypoint}')
    
    def result_callback(self, future):
        result = future.result()
        if result:
            self.get_logger().info('Goal reached successfully.')
            # 한 바퀴 회전 수행
            self.spin_robot()
        else:
            self.get_logger().info('Goal failed.')

    def spin_robot(self):
        # Twist 메시지를 사용해 로봇을 회전
        twist_msg = Twist()
        twist_msg.angular.z = 1.0  # 회전 속도 (라디안/초)
        
        spin_time = 4 * math.pi / twist_msg.angular.z  # 한 바퀴 회전 시간 계산
        start_time = time.time()

        self.get_logger().info('Spinning robot...')
        while time.time() - start_time < spin_time:
            self.cmd_vel_publisher.publish(twist_msg)
            time.sleep(0.1)  # 100ms 간격으로 퍼블리시

        # 회전 멈춤
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info('Spin complete.')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._goal_handle = goal_handle

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Canceling goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('No active goal.')

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled.')
        else:
            self.get_logger().info('Goal cancellation failed.')


def main():
    rclpy.init()
    node = MoveGoal()

    # 초기 위치 설정
    initial_x = 0.1007787823513794
    initial_y = -0.01276879961445528
    initial_yaw = 0.0
    node.publish_initial_pose(initial_x, initial_y, initial_yaw)


    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
