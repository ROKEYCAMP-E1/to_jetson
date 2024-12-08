#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Turtlebot3FeedbackControl(Node):
    def __init__(self):
        super().__init__('turtlebot3_feedback_control')

        # Publisher (AMR에게 명령 보냄)
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber (camera image로부터 명령 받음)
        self.feedback_subscriber = self.create_subscription(
            String, '/feedback_topic', self.feedback_callback, 10
        )

        # Twist message for movement
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.0  # Default linear speed
        self.move_cmd.angular.z = 0.0  # Default angular speed

        self.get_logger().info(
            "Feedback Control Node Started. Listening for feedback on /feedback_topic."
        )

    def feedback_callback(self, msg):
        """피드백 메시지 처리"""
        self.get_logger().info(f"Received feedback: {msg.data}")
        # 피드백 메시지에 따라 동작을 추가적으로 정의

        if "right" in msg.data:  # 오른쪽 시야가 보이도록 회전
            self.get_logger().info("right: 오른쪽으로 회전합니다.")
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = -0.5
        elif "left" in msg.data:  # 왼쪽 시야가 보이도록 회전
            self.get_logger().info("left: 왼쪽으로 회전합니다.")
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.5
        elif "greenzone" in msg.data:#모르는 피드백일 경우 stop
            self.get_logger().info("good tracking")
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
        elif "up" in msg.data:  # 앞(위)으로 이동
            self.get_logger().info("up: 앞으로 이동합니다.")
            self.move_cmd.linear.x = 0.5
            self.move_cmd.angular.z = 0.0

        self.cmd_vel.publish(self.move_cmd)

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
        except KeyboardInterrupt:#컨트롤C로 종료
            self.get_logger().info("Ctrl+C detected. Exiting...")
        finally:
            self.get_logger().info("Shutting down Turtlebot3 Feedback Control Node.")


def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3FeedbackControl()
    try:
        node.run()#노드 시작
    finally:
        node.destroy_node()#노드 종료를 위한 정리
        rclpy.shutdown()


if __name__ == "__main__":
    main()
