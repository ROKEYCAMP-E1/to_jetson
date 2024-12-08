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




class MoveInit(Node):
    def __init__(self):
        super().__init__('move_init')

        # QoS 설정: 속도 위주
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 데이터 손실 가능
            durability=DurabilityPolicy.VOLATILE,      # 연결 중에만 데이터 유지
            depth=10                                   # 최근 10개의 메시지만 유지
        )


        self.action_client=