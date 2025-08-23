import os
# try to silence CTranslate2 warning (if present)
os.environ.setdefault("CT2_VERBOSE", "-1")

import re
import socket
import ollama
from twist_micro_server import twist_api
from functions import listen, say, speak, twist, string_to_twist_essentials
import rclpy
import asyncio
import requests
import time
import inspect

# ---- CONFIG ----
CHK_VOICE = False
SIM = False                      # keep False to use real ESP by default
AUTO_SIM_IF_OFFLINE = True       # if True, fall back to sim when ESP is unreachable
MAX_OFFLINE_SECONDS = 5.0        # consider ESP offline if not reachable for this many seconds
HEALTH_PATH = "/"                # path to probe for minimal health (set to "/" or "/status" if your ESP implements)
ESP_HOST = "192.168.4.1"
ESP_PORT = 80
ESP_ENDPOINT = "/twist"
ESP_TIMEOUT = 3                  # request timeout (seconds)
ESP_RETRIES = 2                  # per-command attempts
ESP_PAUSE = 0.05                 # base pause between retries
SLEEP_AFTER_SPEAK = 0.05
STARTING_SPEECH = "Voice Check... Cheem tapaak dum dum..."
# ----------------

ESP_URL = f"http://{ESP_HOST}:{ESP_PORT}{ESP_ENDPOINT}"
ESP_HEALTH_URL = f"http://{ESP_HOST}:{ESP_PORT}{HEALTH_PATH}"

# Session for keep-alive
_session = requests.Session()
_session.headers.update({"Connection": "keep-alive"})

# Compile regex
_BRACKET_RE = re.compile(r'\[.*?\]')

# --- Utility: quick TCP reachability check ---
def is_esp_reachable(timeout=1.0):
    """Fast TCP connect test to host:port (does not require HTTP)."""
    try:
        sock = socket.create_connection((ESP_HOST, ESP_PORT), timeout=timeout)
        sock.close()
        return True
    except Exception:
        return False

def wait_for_esp(timeout_total=MAX_OFFLINE_SECONDS, poll_interval=0.5):
    """Wait up to timeout_total for the ESP to respond to a TCP connect.
    Returns True if reachable within timeout, otherwise False."""
    start = time.time()
    while time.time() - start < timeout_total:
        if is_esp_reachable(timeout=0.8):
            return True
        time.sleep(poll_interval)
    return False

# --- HTTP send with better backoff & diagnostics ---
def send_to_esp_http_single(lin, ang, dur, use_health_check=True):
    """
    Send a single [lin,ang,dur] payload. Does a quick reachability test first.
    Returns (ok:bool, message:str).
    """
    # Quick TCP check
    if not is_esp_reachable(timeout=0.8):
        msg = f"ESP {ESP_HOST}:{ESP_PORT} not reachable (TCP connect failed)."
        return False, msg

    payload = f'[{lin},{ang},{dur}]'
    attempt = 0
    while attempt < ESP_RETRIES:
        try:
            r = _session.post(ESP_URL, data=payload, timeout=ESP_TIMEOUT)
            if r.status_code == 200:
                return True, r.text.strip()
            else:
                err = f"HTTP {r.status_code}: {r.text.strip()}"
                print("ESP returned:", err)
        except requests.exceptions.ConnectTimeout as e:
            print(f"ConnectTimeout (attempt {attempt+1}):", e)
        except requests.exceptions.ReadTimeout as e:
            print(f"ReadTimeout (attempt {attempt+1}):", e)
        except requests.exceptions.ConnectionError as e:
            print(f"ConnectionError (attempt {attempt+1}):", e)
        except Exception as e:
            print(f"Unexpected error sending to ESP (attempt {attempt+1}):", e)

        attempt += 1
        # exponential-ish backoff but bounded
        time.sleep(ESP_PAUSE * (2 ** attempt))

    return False, f"Failed after {ESP_RETRIES} tries"

# helper: parse bracket blocks into list of triples
def process_action_blocks(action_text):
    cmds = []
    blocks = _BRACKET_RE.findall(action_text)
    for b in blocks:
        try:
            d = string_to_twist_essentials(b)
            if not d or len(d) < 3:
                print("Parser returned unexpected:", d, "for block:", b)
                continue
            lin_val, ang_val, dur_val = float(d[0]), float(d[1]), float(d[2])
            cmds.append((lin_val, ang_val, dur_val))
        except Exception as e:
            print("Failed to parse block:", b, e)
            continue
    return cmds

# wrapper to call say/speak in a way that waits for completion if coroutine
def speak_and_wait(text):
    if not text or not text.strip():
        return
    try:
        if inspect.iscoroutinefunction(say):
            asyncio.run(say(text))
        else:
            result = say(text)
            if inspect.isawaitable(result):
                try:
                    asyncio.run(result)
                except RuntimeError:
                    time.sleep(SLEEP_AFTER_SPEAK)
            else:
                time.sleep(SLEEP_AFTER_SPEAK)
    except Exception as e:
        try:
            if inspect.iscoroutinefunction(speak):
                asyncio.run(speak(text))
            else:
                speak(text)
            time.sleep(SLEEP_AFTER_SPEAK)
        except Exception as e2:
            print("TTS failed:", e, e2)

# simulator runner (used if SIM or fallback)
def sim_twist_batch(cmds):
    for l,a,d in cmds:
        try:
            twist(l,a,d)
        except Exception as e:
            print("SIM twist error:", e)
        time.sleep(max(0.02, ESP_PAUSE))

# MAIN
if CHK_VOICE:
    print("Checking voice...")
    speak_and_wait(STARTING_SPEECH)
    print("Said!")

print(f"Controller started. Using ESP at {ESP_URL}")

try:
    # optionally check once at startup and print a helpful message
    reachable = wait_for_esp(timeout_total=2.0)
    if not reachable:
        print(f"Warning: ESP {ESP_HOST}:{ESP_PORT} did not respond to TCP within 2s.")
        if AUTO_SIM_IF_OFFLINE:
            print("AUTO_SIM_IF_OFFLINE=True -> falling back to simulator unless ESP comes back.")
    else:
        print("ESP seems reachable (TCP connect OK).")

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
            model='bot_annie',
            messages=[{'role': 'user', 'content': USER_PROMPT}],
            stream=True,
        )

        SPEECH = ""
        ACTION = ""
        record_action = False

        for c in stream:
            chunk = c['message']['content']
            if '[' in chunk:
                record_action = True

            if record_action:
                ACTION += chunk
            else:
                SPEECH += chunk

        if SPEECH.strip():
            speak_and_wait(SPEECH)

        parsed_cmds = process_action_blocks(ACTION)
        if parsed_cmds:
            print("Parsed commands:", parsed_cmds)

            # Check reachability just before sending
            if SIM or (not is_esp_reachable() and AUTO_SIM_IF_OFFLINE):
                print("ESP not reachable — using SIM for commands.")
                sim_twist_batch(parsed_cmds)
            elif not is_esp_reachable():
                # ESP unreachable and not allowed to auto-sim: print helpful diagnostic and skip
                print(f"ESP {ESP_HOST}:{ESP_PORT} unreachable, skipping commands. Try ping/curl/serial.")
            else:
                for (lin_val, ang_val, dur_val) in parsed_cmds:
                    print("ACTION parsed:", lin_val, ang_val, dur_val)
                    ok, resp = send_to_esp_http_single(lin_val, ang_val, dur_val)
                    print("Sent:", ok, resp)
                    time.sleep(ESP_PAUSE)
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