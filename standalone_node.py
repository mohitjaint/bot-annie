# This script is responsible to fetch and process the image data from the topic /camera
# =------------------------------=
# Author: Broteen Das


import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class MinimalSubscriber(Node):

    def __init__(self):
        print("Initialising...")

        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.captured_frame_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

        print("Initialisation Complete!")

    def captured_frame_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Image", frame)
        cv2.waitKey(1)  # Display the image for 1 millisecond


        frame : np.ndarray
        # DO YOUR AI WITH THE VARIABLE "frame"...
        

def main(args=None):
    rclpy.init(args=args)
    cam_feedback = MinimalSubscriber()

    rclpy.spin(cam_feedback)
    cam_feedback.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()