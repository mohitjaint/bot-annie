import pyttsx3

engine = pyttsx3.init("festival") # espeak
voices = engine.getProperty('voices')

for voice in voices:
    text = "TESTING"
    print("Using voice:", voice.id)
    engine.setProperty('voice', voice.id)
    engine.say(text)
    engine.runAndWait()
