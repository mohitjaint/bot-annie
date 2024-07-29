#!/usr/bin/env python3

import asyncio
import subprocess
import edge_tts
import ollama


'''
edge-playback --text "Hello, world!"
'''

VOICE = "en-GB-SoniaNeural"

async def speak(TEXT) -> None:

    print("\n\nSaying:", TEXT, "\n\n")


    communicate = edge_tts.Communicate(TEXT, VOICE)
    process = subprocess.Popen(["ffplay", "-nodisp", "-autoexit", "-"], stdin=subprocess.PIPE)
    async for chunk in communicate.stream():
        if chunk["type"] == "audio":
            process.stdin.write(chunk["data"])
    process.stdin.close()
    try:
        await process.wait()
    except TypeError:
        pass




if __name__ == "__main__":

    asyncio.run(speak("SPEECH CHECK"))


    while False:
        TEXT = input(">>> ")

        print("Response:")


        stream = ollama.chat(
            model='bot_stella',
            messages=[{'role': 'user', 'content': TEXT}],
            stream=True,
        )


        speech_txt = ""

        for chunk in stream:
            txtbit = chunk['message']['content']
            print(txtbit, end='', flush=True)
            if '\n' in txtbit:
                print("END!!!!!!!!!!\n\n")
            # Live Action




        print()

