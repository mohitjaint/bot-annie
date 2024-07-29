from transformers import AutoProcessor, BarkModel

import os
os.environ["SUNO_OFFLOAD_CPU"] = "True"
os.environ["SUNO_USE_SMALL_MODELS"] = "True"

processor = AutoProcessor.from_pretrained("suno/bark")
model = BarkModel.from_pretrained("suno/bark")

voice_preset = "v2/en_speaker_6"

print("Initialised!")
inputs = processor("Hello, my dog is cute", voice_preset=voice_preset)

print("Starting generation...")
audio_array = model.generate(**inputs)
audio_array = audio_array.cpu().numpy().squeeze()

print("Done!")
import scipy

sample_rate = model.generation_config.sample_rate
scipy.io.wavfile.write("bark_out.wav", rate=sample_rate, data=audio_array)

# from bark import SAMPLE_RATE, generate_audio, preload_models
# from scipy.io.wavfile import write as write_wav
# from IPython.display import Audio

# # download and load all models
# preload_models()

# # generate audio from text
# text_prompt = """
#      Hello, my name is Suno. And, uh — and I like pizza. [laughs] 
#      But I also have other interests such as playing tic tac toe.
# """
# audio_array = generate_audio(text_prompt)

# # save audio to disk
# write_wav("bark_generation.wav", SAMPLE_RATE, audio_array)
  
# # play text in notebook
# Audio(audio_array, rate=SAMPLE_RATE)