import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
import math


class MoveInit(Node):
    def __init__(self):
        super().__init__('move_init')

        self.current_pose = None
        self.timer = None
        self._goal_handle = None

        # Action client for NavigateToPose
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # AMCL pose subscriber (deferred activation)
        self.amcl_pose_subscriber = None

        # Service terminate subscriber
        self.service_terminate_subscriber = self.create_subscription(
            String,
            '/service_terminate',
            self.terminate_callback,
            10
        )

    def terminate_callback(self, msg):
        """
        Triggered when a service termination message is received.
        Activates AMCL pose subscription to get the current position.
        """
        self.get_logger().info(f"Service termination message received: {msg.data}")

        # Subscribe to AMCL pose to get the current position
        if not self.amcl_pose_subscriber:
            self.amcl_pose_subscriber = self.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose',
                self.amcl_pose_callback,
                10
            )
            self.get_logger().info("Subscribed to AMCL pose to get current position.")
        else:
            self.get_logger().info("Already subscribed to AMCL pose.")

    def amcl_pose_callback(self, msg):
        """
        Callback to receive the current position from AMCL.
        Cancels subscription after receiving the first position.
        """
        if self.current_pose is None:
            self.current_pose = msg
            self.get_logger().info(
                f"AMCL position received: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}"
            )
            # Cancel the AMCL pose subscription after receiving the first position
            self.destroy_subscription(self.amcl_pose_subscriber)
            self.amcl_pose_subscriber = None
            # Proceed to send the robot to the initial position
            self.timer = self.create_timer(10.0, self.send_init_pose)

    def send_init_pose(self):
        """
        Send the robot to the initial position.
        """
        if self.timer:
            self.timer.cancel()
            self.timer = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set the initial position (example coordinates)
        goal_msg.pose.pose.position.x = 0.1007787823513794
        goal_msg.pose.pose.position.y = -0.01276879961445528
        goal_msg.pose.pose.position.z = 0.0

        # Set the orientation
        goal_msg.pose.pose.orientation = self.euler_to_quaternion(0, 0, 0.0)

        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Sending goal to initial position...')

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.current_pose.pose}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return
        self.get_logger().info('Goal accepted.')

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Converts Euler angles to quaternion.
        """
        x = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        y = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        z = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        w = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=x, y=y, z=z, w=w)


def main():
    rclpy.init()
    node = MoveInit()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
