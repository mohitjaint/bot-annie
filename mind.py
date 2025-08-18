import re
import ollama
from twist_micro_server import twist_api
from functions import listen, say, speak, twist, string_to_twist_essentials
import rclpy
import asyncio
import requests
import time

# ---- CONFIG ----
CHK_VOICE = False
SIM = False

# WiFi settings for ESP8266
ESP_HOST = "192.168.4.1"      # set this to ESP IP (if hotspot, check phone hotspot IP range)
ESP_PORT = 80
ESP_ENDPOINT = "/twist"       # endpoint on ESP
ESP_URL = f"http://{ESP_HOST}:{ESP_PORT}{ESP_ENDPOINT}"
ESP_TIMEOUT = 5             # seconds (increase slightly)
ESP_PAUSE = 0.6              # seconds to sleep after each command (reduce ESP timeouts)
ESP_RETRIES = 3             # number of retries for each command (on failure)

STARTING_SPEECH = "Voice Check... Cheem tapaak dum dum..."
# ----------------

# helper: send twist command to ESP via HTTP POST
def send_to_esp_http(lin, ang, dur):
    payload = f'[{lin},{ang},{dur}]'   # matches your ESP parser
    attempt = 0
    while attempt < ESP_RETRIES:
        try:
            r = requests.post(ESP_URL, data=payload, timeout=ESP_TIMEOUT)
            if r.status_code == 200:
                print("ESP OK:", r.text.strip())
                return True
            else:
                print(f"ESP returned {r.status_code}: {r.text.strip()}")
        except Exception as e:
            print("Error sending to ESP (attempt", attempt+1, "):", e)
        # backoff before retry
        attempt += 1
        backoff = ESP_PAUSE * attempt
        print(f"Retrying after {backoff:.2f}s... (attempt {attempt+1}/{ESP_RETRIES})")
        time.sleep(backoff)
    # final failure
    print("Failed to send command to ESP after", ESP_RETRIES, "attempts.")
    return False

# helper: process a string that may contain multiple [ ... ] commands
def process_action_blocks(action_text):
    """
    Finds all bracketed blocks like "[...]" and attempts to parse each
    into (lin, ang, dur) using string_to_twist_essentials().
    Returns list of parsed triples (lin, ang, dur).
    """
    cmds = []
    # non-greedy find of bracketed blocks
    blocks = re.findall(r'\[.*?\]', action_text)
    for b in blocks:
        try:
            d = string_to_twist_essentials(b)    # expects a bracketed command or similar
            if not d or len(d) < 3:
                print("Parser returned unexpected:", d, "for block:", b)
                continue
            lin_val, ang_val, dur_val = float(d[0]), float(d[1]), float(d[2])
            cmds.append((lin_val, ang_val, dur_val))
        except Exception as e:
            print("Failed to parse block:", b, e)
            continue
    return cmds

# Initial Checks___________
if CHK_VOICE:
    print("Checking voice...")
    asyncio.run(speak(STARTING_SPEECH))
    print("Said!")

print("Controller started. Using ESP WiFi at", ESP_URL)

try:
    while True:
        try:
            USER_PROMPT = listen()
        except KeyboardInterrupt:
            input("Stopped Listening! Press enter to resume")
            continue
        except Exception as e:
            print("Error in listening:", e)
            continue

        if USER_PROMPT is None or USER_PROMPT.strip() == '':
            continue

        if USER_PROMPT == '/bye':
            break

        stream = ollama.chat(
            #model='bot_annie_naughty',
            model='bot_annie_naughty_2',

            messages=[{'role': 'user', 'content': USER_PROMPT}],
            stream=True,
        )

        SPEECH = ""
        ACTION = ""
        record_action = False

        for c in stream:
            chunk = c['message']['content']

            # If a chunk contains any bracket open, we enter action-recording mode.
            # We still accumulate everything to ACTION until we hit the end of the stream or we detect full blocks.
            if '[' in chunk:
                record_action = True
                # If we had some speech accumulated, speak it now before action commands.
                if SPEECH.strip():
                    say(SPEECH)
                SPEECH = ""
                ACTION += chunk
                # continue to next chunk to gather potential ending bracket(s)
                continue

            elif ']' in chunk:
                # close of action content (might still be multiple blocks)
                ACTION += chunk

                # process all bracketed blocks inside ACTION
                parsed_cmds = process_action_blocks(ACTION)

                if not parsed_cmds:
                    print("No valid commands found in ACTION:", ACTION)
                    ACTION = ""
                    record_action = False
                    continue

                # Execute each parsed command sequentially, with pause between commands
                for (lin_val, ang_val, dur_val) in parsed_cmds:
                    print("ACTION parsed:", lin_val, ang_val, dur_val)
                    if SIM:
                        try:
                            twist(lin_val, ang_val, dur_val)
                        except Exception as e:
                            print("SIM twist error:", e)
                        # maintain pause so simulator isn't spammed
                        time.sleep(ESP_PAUSE)
                    else:
                        ok = send_to_esp_http(lin_val, ang_val, dur_val)
                        # always pause after an attempt (successful or not) to avoid flooding
                        time.sleep(ESP_PAUSE)
                ACTION = ""
                record_action = False
                continue

            # accumulate speech or in-progress action
            if not record_action:
                SPEECH += chunk
            else:
                ACTION += chunk

        # end of streamed assistant response
        if SPEECH.strip():
            say(SPEECH)

        print()

finally:
    print("Shutting down...")
    try:
        twist_api.destroy_node()
    except:
        pass
    try:
        rclpy.shutdown()
    except:
        pass