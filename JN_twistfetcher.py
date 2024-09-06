from time import sleep
import requests
import serial
import sys

HOST_IP = "192.168.36.245"

if len(sys.argv) > 1:
    if sys.argv[1] == "--remote" or sys.argv[1] == "-r":
        HOST_IP = sys.argv[2]
        print("HOST IP:", HOST_IP)

url = f"http://{HOST_IP}:5500/command.txt"

print("Entering getter loop...")


ARDUINO = None
print("Trying to connect to Arduino @ /dev/ttyACM0...")
try:
    ARDUINO = serial.Serial("/dev/ttyACM0", 9600)
    print("Connected!")
except:
    print("Cannot connect to Arduino... Terminating!")
    exit()

FIRST = True
OLD_STAMP = None
print("Entering fetcher loop...")
while True:
    response = requests.get(url)
    if response.status_code == 200:
        result = response.text

        stamp = result[:result.index("[")]
        command = result[result.index("["):]

        if stamp != OLD_STAMP:
            OLD_STAMP = stamp
            if FIRST:
                FIRST = False
                print("GOOD HEALTH!")
                continue
            print(f"New command received: {command}")
            ARDUINO.write(command.encode())
        

    else:
        print("Error:", response.status_code)
    sleep(0.2)