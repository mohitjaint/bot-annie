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
# ---- CONFIG ----
ESP_TIMEOUT = 3.0                  # Reduced from 3.0 to 1.0 (request timeout)
ESP_RETRIES = 3                    # Increased from 2 to 3 (per-command attempts)
ESP_PAUSE = 0.03                   # Reduced from 0.05 to 0.03 (base pause)
# ----------------             
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
_SENTENCE_END_RE = re.compile(r'[.!?]\s+')

# --- Utility: quick TCP reachability check ---
def is_esp_reachable(timeout=3.0):
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

def send_to_esp_http_single(lin, ang, dur, use_health_check=True):
    global _session
    """
    Send a single [lin,ang,dur] payload. ESP handles timing non-blocking.
    Returns (ok:bool, message:str).
    """
    # Quick TCP check with shorter timeout
    if not is_esp_reachable(timeout=0.5):  # Reduced from 0.8 to 0.5
        msg = f"ESP {ESP_HOST}:{ESP_PORT} not reachable (TCP connect failed)."
        return False, msg

    payload = f'[{lin},{ang},{dur}]'
    attempt = 0
    
    while attempt < ESP_RETRIES:
        try:
            # Use shorter timeout for faster retries
            r = _session.post(ESP_URL, data=payload, timeout=3.0)  # Reduced from 3.0 to 1.0
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
            # Re-establish connection on connection error
            _session = requests.Session()
            _session.headers.update({"Connection": "keep-alive"})
            # Add a small delay before retry
            time.sleep(0.1)
        except Exception as e:
            print(f"Unexpected error sending to ESP (attempt {attempt+1}):", e)

        attempt += 1
        # Shorter, more aggressive retry delays
        time.sleep(0.05 * (2 ** attempt))  # Reduced base delay

    return False, f"Failed after {ESP_RETRIES} tries"

# helper: parse bracket blocks into list of triples
def process_action_blocks(action_text):
    cmds = []
    blocks = _BRACKET_RE.findall(action_text)
    for b in blocks:
        try:
            # Remove LIN/ANG text and split
            parts = re.findall(r'[-+]?\d*\.\d+|\d+', b)
            if len(parts) != 3:
                print("Malformed command, skipping:", b)
                continue
            lin_val, ang_val, dur_val = map(float, parts)
            cmds.append((lin_val, ang_val, dur_val))
        except Exception as e:
            print("Failed to parse block:", b, e)
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

# Process text in chunks of 2 sentences
def process_text_in_chunks(text, chunk_size=2):
    """Split text into chunks of specified number of sentences"""
    if not text.strip():
        return []
    
    # Split into sentences
    sentences = _SENTENCE_END_RE.split(text)
    sentences = [s.strip() for s in sentences if s.strip()]
    
    # Group into chunks
    chunks = []
    current_chunk = []
    
    for sentence in sentences:
        current_chunk.append(sentence)
        if len(current_chunk) >= chunk_size:
            chunks.append(' '.join(current_chunk))
            current_chunk = []
    
    # Add remaining sentences
    if current_chunk:
        chunks.append(' '.join(current_chunk))
    
    return chunks

# MAIN
if CHK_VOICE:
    print("Checking voice...")
    speak_and_wait(STARTING_SPEECH)
    print("Said!")

print(f"Controller started. Using ESP at {ESP_URL}")

try:
    # optionally check once at startup and print a helpful message
    reachable = wait_for_esp(timeout_total=3.0)
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

        speech_buffer = ""
        action_buffer = ""
        in_action_block = False
        speech_chunks = []

        for c in stream:
            chunk = c['message']['content']
            
            # Check if we're entering or exiting an action block
            if '[' in chunk and not in_action_block:
                in_action_block = True
                # Process any accumulated speech before the action
                if speech_buffer.strip():
                    speech_chunks = process_text_in_chunks(speech_buffer, 2)
                    for speech_chunk in speech_chunks:
                        speak_and_wait(speech_chunk)
                    speech_buffer = ""
            
            if in_action_block:
                action_buffer += chunk
                # Check if action block is complete
                if ']' in chunk:
                    # Process the action immediately
                    parsed_cmds = process_action_blocks(action_buffer)
                    if parsed_cmds:
                        print("Parsed commands:", parsed_cmds)
                        
                        if SIM or (not is_esp_reachable() and AUTO_SIM_IF_OFFLINE):
                            print("ESP not reachable — using SIM for commands.")
                            sim_twist_batch(parsed_cmds)
                        elif not is_esp_reachable():
                            print(f"ESP {ESP_HOST}:{ESP_PORT} unreachable, skipping commands.")
                        else:
                            # In the command execution loops, add a small delay:
                            for (lin_val, ang_val, dur_val) in parsed_cmds:
                                print("ACTION parsed:", lin_val, ang_val, dur_val)
                                ok, resp = send_to_esp_http_single(lin_val, ang_val, dur_val)
                                print("Sent:", ok, resp)
                                
                                # Small delay between commands to avoid overwhelming ESP
                                time.sleep(2)  # 50ms delay between commands
                                
                                if not ok:
                                    time.sleep(0.1)  # Slightly longer pause on failure
                    
                    action_buffer = ""
                    in_action_block = False
            else:
                speech_buffer += chunk
                # Check if we have 2 complete sentences
                sentences = _SENTENCE_END_RE.split(speech_buffer)
                complete_sentences = [s for s in sentences if s.strip() and s.strip()[-1] in '.!?']
                
                if len(complete_sentences) >= 2:
                    # Extract the first 2 complete sentences
                    first_two = ' '.join(complete_sentences[:2])
                    speak_and_wait(first_two)
                    
                    # Remove the spoken part from buffer
                    speech_buffer = speech_buffer[len(first_two):].strip()
        
        # Process any remaining speech after the stream ends
        if speech_buffer.strip():
            speech_chunks = process_text_in_chunks(speech_buffer, 2)
            for speech_chunk in speech_chunks:
                speak_and_wait(speech_chunk)
        
        # Process any remaining actions after the stream ends
        if action_buffer.strip() and ']' in action_buffer:
            parsed_cmds = process_action_blocks(action_buffer)
            if parsed_cmds:
                print("Final parsed commands:", parsed_cmds)
                
                if SIM or (not is_esp_reachable() and AUTO_SIM_IF_OFFLINE):
                    print("ESP not reachable — using SIM for commands.")
                    sim_twist_batch(parsed_cmds)
                elif not is_esp_reachable():
                    print(f"ESP {ESP_HOST}:{ESP_PORT} unreachable, skipping commands.")
                else:
                    for (lin_val, ang_val, dur_val) in parsed_cmds:
                        print("ACTION parsed:", lin_val, ang_val, dur_val)
                        ok, resp = send_to_esp_http_single(lin_val, ang_val, dur_val)
                        print("Sent:", ok, resp)
                        
                        # NO waiting here - ESP handles timing non-blocking
                        if not ok:
                            time.sleep(0.1)  # Small pause on failure only
        
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