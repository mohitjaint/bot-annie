from datetime import datetime
import subprocess
from time import sleep
from geometry_msgs.msg import Twist
from twist_micro_server import twist_api
import speech_recognition as sr
import sounddevice as sd
import numpy as np
import subprocess
import os
from vosk import Model, KaldiRecognizer
from faster_whisper import WhisperModel
import tempfile
import scipy.io.wavfile as wav
import re
# import soundfile as sf
# from TTS.api import TTS
# PARAMS
MESSAGE = Twist()
#SPEECH_VOICE = "en-IN-NeerjaNeural"
PIPER_MODEL = os.path.expanduser("piper-voices/en_US-hfc_female-medium.onnx")
##VOSK_MODEL_PATH = os.path.join(os.path.dirname(__file__), "test_area/models/vosk-model-small-en-us-0.15")
##vosk_model = Model(VOSK_MODEL_PATH)
##audio_q = queue.Queue()
model = WhisperModel("tiny.en", device="cpu")
#tts_model = TTS(model_name="tts_models/en/ljspeech/tacotron2-DDC", progress_bar=False)

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

import numpy as np
import sounddevice as sd
import tempfile
import scipy.io.wavfile as wav
import re

def play_beep(frequency=1000, duration=0.2, samplerate=16000):
    """Play a beep sound"""
    t = np.linspace(0, duration, int(samplerate * duration), endpoint=False)
    wave = 0.5 * np.sin(2 * np.pi * frequency * t)
    sd.play(wave, samplerate)
    sd.wait()

def listen(show=True) -> str:
    """Offline speech recognition using faster-whisper (tiny.en) with a beep before recording"""
    duration = 5  # seconds to record
    samplerate = 16000

    # Play beep before listening
    play_beep(frequency=880, duration=0.25)   # tudung-style beep

    print("\rListening...", end="", flush=True)

    # Record audio
    audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype='int16')
    sd.wait()

    # Save to temporary WAV file
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmpfile:
        wav.write(tmpfile.name, samplerate, audio)
        tmp_path = tmpfile.name

    # Transcribe
    segments, _ = model.transcribe(tmp_path)
    text = " ".join([seg.text for seg in segments]).strip()
    
    # Cleanup text (only safe characters)
    text = re.sub(r"[^a-zA-Z0-9 ,.\[\]\-_]", "", text)

    if show and text:
        print("\rYOU:", end=" ")
        print("\033[92m" + text + "\033[0m \n")

    return text

    
# def listen(show=True) -> str:
#     """Offline speech recognition using faster-whisper (tiny.en)"""
#     duration = 5  # seconds to record
#     samplerate = 16000

#     print("\rListening...", end="", flush=True)

#     # Record audio
#     audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype='int16')
#     sd.wait()

#     # Save to temporary WAV file
#     with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmpfile:
#         wav.write(tmpfile.name, samplerate, audio)
#         tmp_path = tmpfile.name

#     # Transcribe
#     segments, _ = model.transcribe(tmp_path)
#     text = " ".join([seg.text for seg in segments]).strip()
    
#     text = re.sub(r"[^a-zA-Z0-9 ,.\[\]\-_]", "", text)

#     if show and text:
#         print("\rYOU:", end=" ")
#         print("\033[92m" + text + "\033[0m \n")

#     return text

    
# def _callback(indata, frames, time, status):
#     if status:
#         print(status, flush=True)
#     audio_q.put(bytes(indata))

# def listen(show=True) -> str:
#     """Listen using Vosk (offline speech recognition)"""
#     print("\rListening...", end="", flush=True)

#     # 16 kHz mono PCM expected by Vosk
#     with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype="int16",
#                            channels=1, callback=_callback):

#         rec = KaldiRecognizer(vosk_model, 16000)
#         result_text = ""

#         while True:
#             data = audio_q.get()
#             if rec.AcceptWaveform(data):
#                 res = json.loads(rec.Result())
#                 result_text = res.get("text", "")
#                 break

#         if show and result_text.strip():
#             print("\rYOU:", end=" ")
#             print("\033[92m" + result_text + "\033[0m \n")

#         return result_text
    
# def listen(show=True) -> str:
#     # obtain audio from the microphone
#     r = sr.Recognizer()
#     with sr.Microphone() as source:
#         print(f"\rListening...", end="", flush=True)
#         audio = r.listen(source, timeout=2, phrase_time_limit=3)

#     print("\rRecognizing...", end="", flush=True)
#     try:
#         # for testing purposes, we're just using the default API key
#         # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
#         # instead of `r.recognize_google(audio)`

#         recog_text = r.recognize_google(audio)
#         # recog_text = r.recognize_sphinx(audio)
#         print("YOU:", end=" ")
#         print("\033[92m" + recog_text + "\033[0m \n")
#         return recog_text
    
    
#     except sr.UnknownValueError:
#         print("Google Speech Recognition could not understand audio")
#         return ""
#     except sr.RequestError as e:
#         print("Could not request results from Google Speech Recognition service; {0}".format(e))
#         return ""




# def preprocess_text_for_tts(text: str) -> str:
#     """
#     Clean text for TTS:
#     - Only allows a-z, A-Z, numbers, space, comma, period, dash, underscore, brackets
#     - Keeps Annie's responses natural (no replacements needed)
#     """
#     return re.sub(r"[^a-zA-Z0-9 ,._\-\[\]]+", "", text)

# def speak(TXT: str):
#     """Use faster TTS (Tacotron2 + HiFiGAN)"""
#     TXT = TXT.strip()
#     if not TXT:
#         return

#     print("\033[92m" + TXT + "\033[0m \n")

#     try:
#         # Generate audio
#         wav = tts_model.tts(TXT)
#         if len(wav) == 0:
#             print(" > Skipping empty audio")
#             return

#         # Play audio
#         sd.play(wav, samplerate=22050)
#         sd.wait()

#     except Exception as e:
#         print(" > TTS error:", e)



# def say(SPEECH: str):
#     """Print and speak text"""
#     print("\033[94m" + SPEECH + "\033[0m")
#     if SPEECH.strip():
#         speak(SPEECH)

    
def say(SPEECH: str):
    print("\033[94m" + SPEECH + "\033[0m")
    if len(SPEECH) != 0:
        speak(SPEECH)

def speak(TXT: str):
    """Use Piper TTS for speech output - only speaks alphabets, numbers, and basic punctuation"""
    # Filter text to only include allowed characters
    allowed_chars = re.compile(r'[^a-zA-Z0-9 .,!?\'\"]')
    filtered_text = allowed_chars.sub('', TXT)
    
    print("\033[92m" + filtered_text + "\033[0m \n")

    if not filtered_text.strip():
        print("No speakable content after filtering")
        return

    try:
        # Generate audio with Piper
        process = subprocess.Popen(
            ["piper", "--model", PIPER_MODEL, "--output-raw"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE
        )

        # Send filtered text to Piper
        process.stdin.write(filtered_text.encode("utf-8"))
        process.stdin.close()

        # Read raw audio (16-bit PCM, 22050Hz mono by default)
        raw_audio = process.stdout.read()
        process.wait()

        # Convert to NumPy array for playback
        audio_data = np.frombuffer(raw_audio, dtype=np.int16).astype(np.float32) / 32768.0

        # Play audio
        sd.play(audio_data, samplerate=22050)
        sd.wait()

    except Exception as e:
        print("Piper TTS error:", e)

# def say(SPEECH: str):
#     print("\033[94m" + SPEECH + "\033[0m")
#     if len(SPEECH) != 0:
#         asyncio.run(speak(SPEECH))

# async def speak(TXT : str):
#     communicate = edge_tts.Communicate(TXT, SPEECH_VOICE, pitch="+20Hz", rate="+20%")
    
    
#     print("\033[92m" + TXT + "\033[0m \n")
    
#     try:
#         process = subprocess.Popen(["ffplay", "-nodisp", "-autoexit", "-"], stdin=subprocess.PIPE)

#         async for chunk in communicate.stream():
#             if chunk["type"] == "audio":
#                 process.stdin.write(chunk["data"])
#         process.stdin.close()
#         try:
#             await process.wait()
#         except TypeError:
#             pass
#     except Exception as e:
#         print("There was an error...", e)

# def string_to_twist_essentials(MSG: str) -> tuple:
#     # [LIN 1.0 ANG 0.0 3]
#     MSG = MSG.strip()

#     if MSG[-1] == ".":
#         MSG = MSG[:-1]

#     print(f"`{MSG}`")
#     if not (MSG[0] == "[" and MSG[-1] == "]"):
#         raise ValueError("Invalid message format. Must be enclosed in square brackets.")
    
#     MSG = MSG.replace("[", "").replace("]", "").split(" ")


#     x = float(MSG[1])
#     z = float(MSG[3])
#     time = float(MSG[4])

#     print("TWIST DETES \nx:", x, "z:", z, "time:", time)

#     return x, z, time
def string_to_twist_essentials(MSG: str) -> tuple:
    MSG = MSG.strip()
    
    # Remove brackets and any surrounding text
    start_idx = MSG.find('[')
    end_idx = MSG.find(']')
    
    if start_idx == -1 or end_idx == -1:
        raise ValueError(f"Invalid Twist command (missing brackets): {MSG}")
    
    # Extract just the content between brackets
    content = MSG[start_idx + 1:end_idx]
    
    # Try comma-separated format first: [x,y,z]
    if ',' in content:
        parts = [part.strip() for part in content.split(',')]
        if len(parts) == 3:
            try:
                x = float(parts[0])
                z = float(parts[1])
                time = float(parts[2])
                print("TWIST DETECTS (comma format) \nx:", x, "z:", z, "time:", time)
                return x, z, time
            except ValueError:
                pass  # Fall through to try the other format
    
    # Try space-separated format: [LIN x ANG z time]
    parts = content.split()
    if len(parts) >= 5 and parts[0] == "LIN" and parts[2] == "ANG":
        try:
            x = float(parts[1])
            z = float(parts[3])
            time = float(parts[4])
            print("TWIST DETECTS (LIN/ANG format) \nx:", x, "z:", z, "time:", time)
            return x, z, time
        except (ValueError, IndexError):
            pass
    
    # If neither format works, try to extract any three numbers in sequence
    import re
    numbers = re.findall(r"[-+]?\d*\.\d+|\d+", content)
    if len(numbers) >= 3:
        try:
            x = float(numbers[0])
            z = float(numbers[1])
            time = float(numbers[2])
            print("TWIST DETECTS (number extraction) \nx:", x, "z:", z, "time:", time)
            return x, z, time
        except ValueError:
            pass
    
    raise ValueError(f"Could not parse Twist values from: {content}")