from time import sleep

RESPONSE = "Sure, let me give you an example of a quick spin to the left! [LIN 0.0 ANG -0.4 1]. Now, how about I do a slow forward roll? [LIN 0.2 ANG 0.0 10] And finally, if you're feeling adventurous, let me show you how it feels to be upside down! [LIN 0.3 ANG 0.5 8]."

SPEECH = ""
ACTION = ""
record_action = False
for chunk in RESPONSE.split(' '):
    # print(chunk, end=' ', flush=True)
    if '[' in chunk:
        # Incoming action! Speak whats to speak and act
        record_action = True
        print(SPEECH)
        # speak(SPEECH)

        SPEECH = ""

    elif ']' in chunk:
        record_action = False
        # Send Twist here
        ACTION += chunk
        print("ACT:", ACTION)
        ACTION = ""
        continue

    if not record_action:
        SPEECH += chunk + ' '
    else:
        ACTION += chunk + ' '


    sleep(0.05)