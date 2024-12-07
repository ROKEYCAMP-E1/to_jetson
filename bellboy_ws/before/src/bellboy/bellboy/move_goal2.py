import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math, sys


class MoveGoal(Node):
    def __init__(self):
        super().__init__('move_goal')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None

        # 초기 위치 퍼블리셔 추가
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

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
    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # 목표 위치 설정
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # 목표 방향 설정
        goal_msg.pose.pose.orientation = self.euler_to_quaternion(0, 0, yaw)

        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Sending goal...')

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._goal_handle = goal_handle

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.current_pose.pose}')

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Canceling goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('No active goal.')

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_cancelled) > 0:
            self.get_logger().info('Goal successfully canceled. Exiting...')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
        else:
            self.get_logger().info('Goal cancellation failed.')


def main():
    rclpy.init()
    node = MoveGoal()

    # 초기 위치 설정
    initial_x = -0.04410728055892729
    initial_y = -0.08694220233498898
    initial_yaw = 0.0
    node.publish_initial_pose(initial_x, initial_y, initial_yaw)

    # 목표 좌표 설정
    goal_x = -1.2978199371227226
    goal_y = -0.24750621448314233
    goal_yaw = 0.0
    node.send_goal(goal_x, goal_y, goal_yaw)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
