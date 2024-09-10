import ollama
from twist_micro_server import twist_api
from functions import listen, say, speak, ard_twist, twist, string_to_twist_essentials
import rclpy
import asyncio
import serial

CHK_VOICE = False
SIM = False

STARTING_SPEECH = "Voice Check... Cheem tapaak dum dum..."
ARDUINO = None

# Initial Checks___________
# Speech
if CHK_VOICE:
    print("Checking voice...")
    asyncio.run(speak(STARTING_SPEECH))
    print("Said!")

if not SIM and ARDUINO is None:
    print("Trying to connect to Arduino @ /dev/ttyACM0...")
    try:
        ARDUINO = serial.Serial("/dev/ttyACM0", 9600)
        print("Connected!")
    except:
        pass
        # say("Cannot connect to Arduino.")

# Movement

while True:

    # USER_PROMPT = input(">> ")
    try:
        USER_PROMPT = listen()
    except KeyboardInterrupt:
        input("Stopped Listening! Press enter to resume")
        continue
    except:
        print("Error in listening")
        continue

    if USER_PROMPT.strip() == '':
        continue

    if USER_PROMPT == '/bye':
        break

    stream = ollama.chat(
        model='bot_baldev',
        messages=[{'role': 'user', 'content': USER_PROMPT}],
        stream=True,
    )

    # for chunk in stream:
    #     print(chunk['message']['content'], end='', flush=True)
    SPEECH = ""
    ACTION = ""
    record_action = False
    for c in stream:
        chunk = c['message']['content']
        if '[' in chunk:
            # Incoming action! Speak whats to speak and act
            record_action = True

            say(SPEECH)
            # asyncio.run(speak(SPEECH))

            SPEECH = ""

        elif ']' in chunk:
            record_action = False
            ACTION += chunk
            
            # Send Twist here
            d = string_to_twist_essentials(ACTION)
            
            if SIM:
                twist(d[0], d[1], d[2])
            else:
                ard_twist(d[0], d[1], d[2], ARDUINO, True)

            ACTION = ""
            continue

        if not record_action:
            SPEECH += chunk
        else:
            ACTION += chunk

    say(SPEECH)
    
    print()

print("Shutting down...")

twist_api.destroy_node()
rclpy.shutdown()