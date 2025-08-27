# Annie Bot 🤖🎙️

Annie is a playful differential-drive bot powered by local LLMs.  
She listens to you, responds with voice, and moves around (or simulates if ESP is offline).  

The "brains" of Annie are powered by an Ollama-hosted model (`bot_annie`), which generates both speech and motor commands (`[lin, ang, dur]`).  
Commands are then sent to the ESP (or simulator) to drive the robot.

---

## ✨ Features
- **Voice-controlled** – listens to you via microphone, transcribes offline with Whisper.
- **Talks back** – uses [Piper TTS](https://github.com/rhasspy/piper) for speech output.
- **Moves around** – generates ROS `Twist` commands and sends them to ESP8266/ESP32 via WiFi.
- **Local LLM** – runs with Ollama (`ollama run bot_annie`) for reasoning + motion planning.
- **Simulator fallback** – if ESP is unreachable, Annie will still run in simulation mode.

---

## 📦 Requirements

### Python libraries
Install Python dependencies:
```bash
pip install -r requirements.txt

```
## System / external dependencies

### ROS 2 (for rclpy and geometry_msgs)
- Example (Ubuntu 22.04, ROS2 Humble):
```bash
sudo apt install ros-humble-rclpy ros-humble-geometry-msgs
```

### Piper TTS (for speech output)
- Install from Piper releases.
- Create a piper-voices folder in the root.
- Download and place the .onnx voice model in:

    piper-voices/en_US-hfc_female-medium.onnx

## 🧠 Model Setup

- Install Ollama first
- Pull Mistral using ollama 
    ```bash
    ollama pull mistral
    ```
- Then build Annie’s custom model:
    ```bash
    ollama create bot_annie -f ./bot_annie_model_file
    ```

- Test it:
    ```bash
    ollama run bot_annie
    ```

## 🚀 Running Annie

- Run the main brain:
    ```bash
    python mind.py
    ```

- Annie will listen, speak, and move.

    - If ESP is connected → sends movement commands.

    - If ESP not reachable → falls back to simulation.

## 📡 ESP Setup

    ESP default WiFi SSID: Annie

    Password: 12345678

- You can change these in the ESP code (/esp_code/).

- ESP must be connected to the same laptop WiFi to receive commands.

## ⏹ Stopping Annie

- Simply press "CTRL + Z" to stop the bot.



## 🛠️ Credits

- Whisper (faster-whisper) – offline STT

- Piper – local TTS

- Ollama – local LLM engine

- ROS 2 – robot communication