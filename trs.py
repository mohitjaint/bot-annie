from geometry_msgs.msg import Twist
from twist_micro_server import twist_api
from time import sleep

def string_to_twist_essentials(MSG: str) -> Twist:
    # [LIN 1.0 ANG 0.0 3]
    MSG = MSG.replace("[", "").replace("]", "").split(" ")

    x = float(MSG[1])
    z = float(MSG[3])
    time = float(MSG[4])

    print("x:", x, "z:", z, "time:", time)


string_to_twist_essentials("[LIN 1.0 ANG 0.0 3]")