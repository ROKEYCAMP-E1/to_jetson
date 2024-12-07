#Control_Node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped,Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

import math,sys

    


class Move_goal(Node):
    def __init__(self):
        super().__init__('move_goal')
        self.action_client= ActionClient(self,NavigateToPose,'navigate_to_pose')
        self._goal_handle = None
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)


    #오일러각을 쿼터니언으로 전환
    #roll=x축 회전, pitch=y축 회전,yaw=z축 회전
    def euler_to_quaternion(self, roll, pitch, yaw):
        x = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        y = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        z = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        w = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=x, y=y, z=z, w=w)

    def publish_initial_pose(self):
        # Create the initial pose message
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'  # The frame in which the pose is defined
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        initial_pose.pose.pose.position.x = 0.3076742798897024 # X-coordinate
        initial_pose.pose.pose.position.y = -0.2026723372726634623 # Y-coordinate
        initial_pose.pose.pose.position.z = 0.0  # Z should be 0 for 2D navigation

        # Set the orientation (in quaternion form)
        initial_pose.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=-0.7066681784808516,  # 90-degree rotation in yaw (example)
            w=0.7075451120052736 # Corresponding quaternion w component
        )

        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]
        self.publisher.publish(initial_pose)
        self.get_logger().info('Initial pose published.')


    #Nav2에 목표좌표와,각 전달
    def send_goal(self):
        goal_msg=NavigateToPose.Goal()
        # goal_msg.pose.header.frame_id='map'
        # goal_msg.pose.header.stamp= self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x=-1.2978199371227226
        goal_msg.pose.pose.position.y=-0.24750621448314233
        goal_msg.pose.pose.position.z=0.0

        
        goal_yaw=1.0
        
        
        goal_msg.pose.pose.orientation = self.euler_to_quaternion(0, 0, goal_yaw)
        self.get_logger().info('Waiting action server')
        self.action_client.wait_for_server()
        self.get_logger().info('goal sending')

        self.send_goal_future=self.action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal reject.')
            return

        self.get_logger().info('Goal accept.')
        self._goal_handle = goal_handle  

    def feedback_callback(self,feedback_msg):
        self.get_logger().info(f'feedback:{feedback_msg.feedback.current_pose.pose}')
    

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('canceling')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

        else:
             self.get_logger().info('No active')

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_cancelled) > 0:
            self.get_logger().info('Exit')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)  # Exit the program after successful cancellation
        else:
            self.get_logger().info('cancel fail')    

def main():
    rclpy.init()
    node = Move_goal()
    node.publish_initial_pose()
    node.send_goal()

    rclpy.spin(node)

if __name__ == '__main__':
    main()  
