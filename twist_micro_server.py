from twist_api import TwistAPI
import rclpy
from threading import Thread


TARGET_TOPIC = "/model/vehicle_blue/cmd_vel"


rclpy.init()
twist_api = TwistAPI(target_topic=TARGET_TOPIC)

def t_api_spinner():
    rclpy.spin(twist_api)

tapi_thread = Thread(target=t_api_spinner)
tapi_thread.start()

print("Twist API Node started!")