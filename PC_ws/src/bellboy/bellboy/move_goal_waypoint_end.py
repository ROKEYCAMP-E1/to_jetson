#move_goal_waypoint2.py
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


class MoveWaypointGoal(Node):
    def __init__(self):
        super().__init__('Move_Waypoint_Goal')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 데이터 손실 가능
            durability=DurabilityPolicy.VOLATILE,      # 연결 중에만 데이터 유지
            depth=10                                   # 최근 10개의 메시지만 유지
        )

        self._goal_handle = None
        self.goal_sent = False  # Goal이 한 번만 전송되도록 관리
        self.goal_canceled = False  # Goal이 한 번만 취소되도록 관리

        self.detect_person = False
        self.detect_luggage = False

        self.timer = None  # 타이머 객체

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

        self.action_client = ActionClient(
            self, FollowWaypoints, 
            '/follow_waypoints'
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


        self.ServiceTerminate_publisher = self.create_publisher(
            String, 
            '/ServiceTerminate', 
            10
        )


    def detect_luggage_callback(self, msg):
        """DetectLuggage 콜백 함수: Luggage를 감지하면 액션 실행"""
        self.get_logger().info(f"Received DetectLuggage message: {msg.data}")
        
        if msg.data:  # 감지된 경우
            self.detect_luggage = True
            self.get_logger().info("Luggage detected. Starting action.")
            self.send_goal()  # 액션 실행

    

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        # 위치 설정
        msg.pose.pose.position.x = 0.1007787823513794
        msg.pose.pose.position.y =-0.01276879961445528
        msg.pose.pose.position.z = 0.0

        # 방향 설정 (쿼터니언)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

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
        self.get_logger().info("Initial pose published")



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

        
        waypoint1.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        waypoints.append(waypoint1)


        waypoint2 = PoseStamped()
        waypoint2.header.stamp.sec = 0
        waypoint2.header.stamp.nanosec = 0
        waypoint2.header.frame_id = "map"  # 프레임 ID를 설정 (예: "map")
        waypoint2.pose.position.x = -1.2978199371227226
        waypoint2.pose.position.y = -0.24750621448314233
        waypoint2.pose.position.z = 0.0

        waypoint2.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

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
        self.timer = self.create_timer(120.0,self.check_person_timeout)


    def check_person_timeout(self):
        if not self.detect_person:  # DetectPerson 값이 False이면
            self.get_logger().info('120 seconds passed, no person detected. Terminating service.')

            # ServiceTerminate 메시지 발행
            terminate_msg = String()
            terminate_msg.data = "AMR"
            self.ServiceTerminate_publisher.publish(terminate_msg)
            self.get_logger().info('Published ServiceTerminate message: AMR.')

            # 타이머 삭제
            self.destroy_timer(self.timer)
            self.cancel_goal()
        else:
            self.get_logger().info('Person detected within 30 seconds. No ServiceTerminate needed.')
            self.destroy_timer(self.timer)  # 타이머 삭제

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._goal_handle = goal_handle
        self._goal_handle.get_result_async().add_done_callback(self.result_callback)


    def feedback_callback(self, feedback_msg):
        # Waypoint 도달 피드백 확인
        current_waypoint_index = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Currently moving to waypoint: {current_waypoint_index}')




    def result_callback(self, future):
        result = future.result()
        if not result:
            self.get_logger().info('No result received or goal was rejected.')
            return

        self.get_logger().info(f'Action result received: {result}')
        self.get_logger().info('Goal reached. Starting spin at goal...')
        
        # 목표 지점에서 회전 실행
        self.spin_robot()

        # 모든 목표 완료 후 Goal 취소
        self.cancel_goal()


    def spin_robot(self):
        twist = Twist()
        twist.linear.x = 0.0  # 직진 속도 (0: 직진하지 않음)
        twist.angular.z = 0.5  # 회전 속도 (라디안/초)
        angular_speed = twist.angular.z  # 라디안/초
        rotation_time = 2 * math.pi / angular_speed  # 360도 회전 시간

        start_time = time.time()
        while time.time() - start_time < rotation_time:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)  # 100ms 간격으로 퍼블리시

        # 회전 멈춤
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Spin complete.')


    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Canceling goal...')
            if not self._goal_handle.status:
                self.get_logger().info('Goal is not active or already completed.')
            return

            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('No active goal.')

    def detect_person_callback(self, msg):
        self.get_logger().info(f"Received detection message: {msg.data}")
        
        if msg.data and not self.goal_canceled:
            self.detect_person = True
            self.get_logger().info("Person detected. Canceling goal...")
            if self._goal_handle is None:
                self.get_logger().info("No active goal to cancel.")
            else:
                self.cancel_goal()
            self.goal_canceled = True

    def cancel_done_callback(self, future):
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info('Goal successfully canceled.')
            else:
                self.get_logger().info('Goal cancellation failed. Proceeding to shutdown.')
        except Exception as e:
            self.get_logger().error(f"Error during goal cancellation: {e}")

        # 노드 종료
        self.shutdown_node()

    def shutdown_node(self):
        self.get_logger().info('Shutting down node...')
        rclpy.shutdown()




def main():
    rclpy.init()
    node = MoveWaypointGoal()

    # 초기 위치 설정

    node.publish_initial_pose()


    rclpy.spin(node)
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
