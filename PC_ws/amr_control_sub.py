#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import select
import termios
import tty
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class Turtlebot3KeyboardControl(Node):
    def __init__(self):
        super().__init__('turtlebot3_keyboard_control')

        # QoS 설정: feedback_subscriber에서만 적용
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 데이터 손실 가능
            durability=DurabilityPolicy.VOLATILE,      # 연결 중에만 데이터 유지
            depth=10                                   # 최근 10개의 메시지만 유지
        )

        # cmd_vel 퍼블리셔 (기본 QoS 사용)
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # feedback_subscriber 구독자 (사용자 지정 QoS 사용)
        self.feedback_subscriber = self.create_subscription(
            String, '/feedback_topic', self.feedback_callback, qos_profile
        )

        # Twist message for movement
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.0  # Default linear speed
        self.move_cmd.angular.z = 0.0  # Default angular speed

        self.get_logger().info(
            "Keyboard Control Node Started. "
            "Use i/k to move forward/backward, j/l to rotate left/right. "
            "Press Ctrl+C to exit."
        )

        # Start keyboard input thread
        self.input_thread = threading.Thread(target=self.keyboard_input_loop, daemon=True)
        self.input_thread.start()

    def feedback_callback(self, msg):
        """피드백 메시지 처리"""
        self.get_logger().info(f"Received feedback: {msg.data}")
        # 피드백 메시지에 따라 동작을 추가적으로 정의

        if "right" in msg.data:  # 오른쪽 시야가 보이도록 회전
            self.get_logger().info("right: 오른쪽으로 회전합니다.")
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = -0.2
            self.cmd_vel.publish(self.move_cmd)
        elif "left" in msg.data:  # 왼쪽 시야가 보이도록 회전
            self.get_logger().info("left: 왼쪽으로 회전합니다.")
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.2
            self.cmd_vel.publish(self.move_cmd)
        elif "up" in msg.data:
            self.get_logger().info("up: 앞(위)으로 이동합니다.")
            self.move_cmd.linear.x = 0.05
            self.move_cmd.angular.z = 0.0
            self.cmd_vel.publish(self.move_cmd)
        elif "down" in msg.data:
            self.get_logger().info("down: 뒤(아래)로 이동합니다.")
            self.move_cmd.linear.x = -0.05
            self.move_cmd.angular.z = 0.0
            self.cmd_vel.publish(self.move_cmd)

    def keyboard_input_loop(self):
        while rclpy.ok():
            key = self.get_key()
            if key:
                self.process_key(key)

    def get_key(self):
        original_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            key = None
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        return key

    def process_key(self, key):
        if key == 'i':
            self.move_cmd.linear.x = 0.1  # Move forward
            self.move_cmd.angular.z = 0.0
        elif key == 'k':
            self.move_cmd.linear.x = 0.0  # Stop
            self.move_cmd.angular.z = 0.0
        elif key == ',':
            self.move_cmd.linear.x = -0.1  # Move backward
            self.move_cmd.angular.z = 0.0
        elif key == 'j':
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.25  # Rotate left
        elif key == 'l':
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = -0.25  # Rotate right
        elif key == '\x03':  # Ctrl+C
            raise KeyboardInterrupt

        self.cmd_vel.publish(self.move_cmd)

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
        except KeyboardInterrupt:
            self.get_logger().info("Ctrl+C detected. Exiting...")
        finally:
            self.get_logger().info("Shutting down Turtlebot3 Keyboard Control Node.")


def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3KeyboardControl()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
