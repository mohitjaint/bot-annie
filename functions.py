from datetime import datetime
import asyncio
import subprocess
from time import sleep
from geometry_msgs.msg import Twist
# from twist_micro_server import twist_api
import edge_tts
import speech_recognition as sr
import serial

# PARAMS
# MESSAGE = Twist()
SPEECH_VOICE = "en-IN-NeerjaNeural"
# SPEECH_VOICE = "en-GB-SoniaNeural"


# List of actions...


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

def ard_twist(x: float, z: float, time: float, arduino=None, remote=False):

    if remote:
        with open('command.txt', 'w') as f:
            f.write(f"{datetime.now().timestamp()}[{x},{z},{time}]")
    elif arduino is not None:
        arduino.write(f"[{x},{z},{time}]".encode())
    else:
        raise "No arduino or remote connection specified."

# ard_twist(1.0, 0.0, 3.0, None, True)

def twist(x: float, z: float, time: float):
    '''Moves the bot in the specified linear x and angular z direction for the specified time. The bot will move infinitely if time is negative.'''


    MESSAGE.linear.x = x
    MESSAGE.linear.y = 0.0
    MESSAGE.linear.z = 0.0

    MESSAGE.angular.x = 0.0
    MESSAGE.angular.y = 0.0
    MESSAGE.angular.z = z
    
    twist_api.send_twist(MESSAGE)

    if time < 0:
        return
    
    sleep(time)

    idle()

def listen(show=True) -> str:
    # obtain audio from the microphone
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print(f"\rListening...", end="", flush=True)
        audio = r.listen(source, timeout=2, phrase_time_limit=3)

    print("\rRecognizing...", end="", flush=True)
    try:
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        # instead of `r.recognize_google(audio)`

        recog_text = r.recognize_google(audio)
        # recog_text = r.recognize_sphinx(audio)
        print("YOU:", end=" ")
        print("\033[92m" + recog_text + "\033[0m \n")
        return recog_text
    
    
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
        return ""
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))
        return ""

def say(SPEECH: str):
    print("\033[94m" + SPEECH + "\033[0m")
    if len(SPEECH) != 0:
        asyncio.run(speak(SPEECH))

async def speak(TXT : str):
    communicate = edge_tts.Communicate(TXT, SPEECH_VOICE, pitch="+20Hz", rate="+20%")
    
    
    print("\033[92m" + TXT + "\033[0m \n")
    
    try:
        process = subprocess.Popen(["ffplay", "-nodisp", "-autoexit", "-"], stdin=subprocess.PIPE)

        async for chunk in communicate.stream():
            if chunk["type"] == "audio":
                process.stdin.write(chunk["data"])
        process.stdin.close()
        try:
            await process.wait()
        except TypeError:
            pass
    except Exception as e:
        print("There was an error...", e)

def string_to_twist_essentials(MSG: str) -> tuple:
    # [LIN 1.0 ANG 0.0 3]
    MSG = MSG.strip()

    if MSG[-1] == ".":
        MSG = MSG[:-1]

    print(f"`{MSG}`")
    if not (MSG[0] == "[" and MSG[-1] == "]"):
        raise ValueError("Invalid message format. Must be enclosed in square brackets.")
    
    MSG = MSG.replace("[", "").replace("]", "").split(" ")


    x = float(MSG[1])
    z = float(MSG[3])
    time = float(MSG[4])

    print("TWIST DETES \nx:", x, "z:", z, "time:", time)

    return x, z, time