from time import sleep
from twist_api import TwistAPI
from geometry_msgs.msg import Twist
import rclpy
from threading import Thread

'''rclpy.init()
tapi = TwistAPI(target_topic="/model/vehicle_blue/cmd_vel")

def t_api_spinner():
    rclpy.spin(tapi)

tapi_thread = Thread(target=t_api_spinner)
tapi_thread.start()

'''
"""
from twist_micro_server import twist_api

sample_msg = Twist() # sample twist message that makes the car go 360

sample_msg.angular.z = 0.2
twist_api.send_twist(sample_msg)
print("First message sent!")
sleep(5)

sample_msg.angular.z = -1.0
twist_api.send_twist(sample_msg)
print("Second message sent!")
sleep(5)

sample_msg.angular.z = 0.0
twist_api.send_twist(sample_msg)
print("Third message sent!")
"""

from twist_micro_server import twist_api

from functions import *

move("w", 1.0, 0.2) # Fails for some reason


# rotate(True, 1.0, 1) # Rotates clockwise for 1 second with a speed of 1.0


move("s", 1.0, 2) # Moves forward for 2 seconds with a speed of 1.0
move("w", 1.5, 3) # Moves forward for 2 seconds with a speed of 1.0


sleep(1) # Required to stop the bot from moving infinitely

# while True:
#     try:
#         sleep(1)
#         print("Still Alive...")

#     except KeyboardInterrupt:
#         print("Breakin...")
#         break

twist_api.destroy_node()
rclpy.shutdown()