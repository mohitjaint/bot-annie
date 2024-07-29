from rclpy.node import Node
from geometry_msgs.msg import Twist 

class TwistAPI(Node):
    def __init__(self, target_topic: str) -> None:
        super().__init__('twist_api')
        
        self.target_topic = target_topic
        self.twist_publisher_ = self.create_publisher(Twist, self.target_topic, 10) # 10 is queue size
        self.timer = self.create_timer(0.5, self.twist_checknsend_callback) # Callback every 0.5 seconds

        self.msg2send : Twist = None


    def twist_checknsend_callback(self):
        # Publish self.msg2send

        if self.msg2send is not None:
            self.twist_publisher_.publish(self.msg2send)
            print("Published message!")
            self.msg2send = None

    def send_twist(self, twist_msg : Twist):
        self.msg2send = twist_msg
