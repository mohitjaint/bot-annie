import ollama
from twist_micro_server import twist_api
from functions import twist, speak
import rclpy
import asyncio


BREAK = False


# Initial Checks___________
# Speech
asyncio.run(speak("Voice check... 1,2,3, bananas will destroy the world!"))
print("Said!")

# Movement


# quit()
while True:

    USER_PROMPT = input(">> ")
    if USER_PROMPT == '/bye':
        break

    stream = ollama.chat(
        model='bot_bheem',
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
            # print(SPEECH)
            speak(SPEECH)

            SPEECH = ""

        elif ']' in chunk:
            record_action = False
            # Send Twist here
            ACTION += chunk
            print("ACT:", ACTION)
            ACTION = ""
            continue

        if not record_action:
            SPEECH += chunk
        else:
            ACTION += chunk


    # print(SPEECH)
    if len(SPEECH) != 0:
        asyncio.run(speak(SPEECH))


    print()

print("Shutting down...")

twist_api.destroy_node()
rclpy.shutdown()