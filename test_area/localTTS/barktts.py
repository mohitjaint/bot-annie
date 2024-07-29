from transformers import AutoProcessor, BarkModel
import scipy
import torch
torch.cuda.empty_cache()


processor = AutoProcessor.from_pretrained("suno/bark")
model = BarkModel.from_pretrained("suno/bark")

model.to("cuda")

def generate_audio(text, preset, output):
    inputs = processor(text, voice_preset=preset)

    for k, v in inputs:
        inputs[k] = v.to("cuda")
    
    audio_array = model.generate(**inputs)
    audio_array = audio_array.cpu().numpy().squeeze()
    sample_rate = model.generation_config.sample_rate

    scipy.io.wavfile.write(output, rate=sample_rate, data=audio_array)

generate_audio("Whattup?", "v2/en_speaker_6", "output.wav")