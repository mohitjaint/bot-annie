import sounddevice as sd
import numpy as np
import tempfile
import scipy.io.wavfile as wav
from faster_whisper import WhisperModel

model = WhisperModel("tiny.en", device="cpu")

duration = 5
samplerate = 48000
device = 5  # your USB mic card number from arecord -l

print("🎙️ Recording...")
audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype="int16", device=device)
sd.wait()
print("✅ Recording finished")

# Save temporary WAV
with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmpfile:
    wav.write(tmpfile.name, samplerate, audio)
    tmp_path = tmpfile.name

# Transcribe
segments, _ = model.transcribe(tmp_path)
text = " ".join([seg.text for seg in segments]).strip()
print("YOU:", text)
