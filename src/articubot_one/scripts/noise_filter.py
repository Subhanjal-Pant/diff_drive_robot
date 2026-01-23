import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist



class NoiseFilter(Node):
    super().__init__('noise_filter')
    self.sub=self.create_subscription(Twist, '/cmd_vel_raw')