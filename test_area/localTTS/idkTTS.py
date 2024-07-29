'''
tts --text "Text for TTS" --model_name "tts_models/multilingual/multi-dataset/bark" --out_path output.wav

https://youtu.be/EyzRixV8s54?t=439
'''

import torch
from TTS.api import TTS

# from TTS.tts.models.bark import Bark
# from TTS.tts.configs.tortoise_config import TortoiseConfig

# config = TortoiseConfig(output_path="manual_output.wav")
# model = Bark(config=config)


# MODEL = "tts_models/multilingual/multi-dataset/xtts_v2"
# MODEL = "tts_models/multilingual/multi-dataset/bark" # NEEDS TOO MUCH RESOURCES ~ 12 GB VRAM
MODEL = "tts_models/en/ljspeech/neural_hmm" # Works | meh!
# MODEL = "tts_models/en/vctk/fast_pitch" # horseshit!

TEXT = "This is gonna be great. I am finally gonna have a voice to speak up yo! [laughs] I am so excited! [pouts] Dunno if I can express myself enough though..."


# Get device
device = "cuda" if torch.cuda.is_available() else "cpu"

# List available 🐸TTS models
# print(TTS().list_models())

# Init TTS
tts = TTS(MODEL).to(device)

# Run TTS
# ❗ Since this model is multi-lingual voice cloning model, we must set the target speaker_wav and language
# Text to speech list of amplitude values as output
# wav = tts.tts(text="Hello world!", speaker_wav="my/cloning/audio.wav", language="en")
# Text to speech to a file
# tts.tts_to_file(text="Hello world!", speaker_wav="my/cloning/audio.wav", language="en", file_path="output.wav")
tts.tts_to_file(text=TEXT, file_path="output.wav")

