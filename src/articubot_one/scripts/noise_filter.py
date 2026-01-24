import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist



class NoiseFilter(Node):
    def __init__(self):
        super().__init__('noise_filter')
        self.sub=self.create_subscription(Twist, '/cmd_vel_raw',self.callback,10)
        self.pub=self.create_publisher(Twist,"/cmd_vel", 10)
    
    
    def callback(self, msg):
        new_msg=Twist()
        new_msg.linear.x=msg.linear.x if(abs(msg.linear.x))>0.01 else 0.0
        new_msg.angular.z=msg.angular.z if(abs(msg.angular.z))>0.01 else 0.0
        self.pub.publish(new_msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node=NoiseFilter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__=='__main__':
    main()