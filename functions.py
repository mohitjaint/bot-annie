import subprocess
from time import sleep
from geometry_msgs.msg import Twist
from twist_micro_server import twist_api
import edge_tts

# List of actions...

MESSAGE = Twist()

SPEECH_VOICE = "en-GB-SoniaNeural"





def idle():
    MESSAGE.linear.x = 0.0
    MESSAGE.linear.y = 0.0
    MESSAGE.linear.z = 0.0

    MESSAGE.angular.x = 0.0
    MESSAGE.angular.y = 0.0
    MESSAGE.angular.z = 0.0

    twist_api.send_twist(MESSAGE)

def move(direction: str, speed: float, time: float):
    '''Moves the bot in the specified direction for the specified time. Forward is "w", backward is "s". The bot will move infinitely if time is negative.'''

    idle()

    if direction == "w":
        MESSAGE.linear.x = speed
    elif direction == "s":
        MESSAGE.linear.x = -speed
    else:
        raise ValueError("Invalid direction. Use 'w' for forward and 's' for backward.")
    
    twist_api.send_twist(MESSAGE)

    if time < 0:
        return
    
    sleep(time)

    idle()

def rotate(clockwise: bool, speed: float, time: float):
    '''Rotates the bot in the specified direction for the specified time. Clockwise is True, counter-clockwise is False.'''

    idle()

    if clockwise:
        MESSAGE.angular.z = -speed
    else:
        MESSAGE.angular.z = speed

    twist_api.send_twist(MESSAGE)


    if time < 0:
        return
    

    sleep(time)
    idle()



############################################################

def twist(x: float, z: float, time: float):
    '''Moves the bot in the specified linear x and angular z direction for the specified time. The bot will move infinitely if time is negative.'''


    MESSAGE.linear.x = x
    MESSAGE.linear.y = 0
    MESSAGE.linear.z = 0

    MESSAGE.angular.x = 0
    MESSAGE.angular.y = 0
    MESSAGE.angular.z = z
    
    twist_api.send_twist(MESSAGE)

    if time < 0:
        return
    
    sleep(time)

    idle()


async def speak(TXT : str):
    communicate = edge_tts.Communicate(TXT, SPEECH_VOICE)
    
    
    print("\033[92m" + TXT + "\033[0m \n")
    

    process = subprocess.Popen(["ffplay", "-nodisp", "-autoexit", "-"], stdin=subprocess.PIPE)

    async for chunk in communicate.stream():
        if chunk["type"] == "audio":
            process.stdin.write(chunk["data"])
    process.stdin.close()
    try:
        await process.wait()
    except TypeError:
        pass