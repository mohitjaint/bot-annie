# sudo chmod a+rw /dev/ttyUSB0

import serial

def write_char_to_serial(char):
    """Writes a single character to /dev/ttyACM0.

    Args:
        char: The character to write.
    """

    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

    try:
        ser.write(char.encode('ascii'))
    except serial.SerialException as e:
        print(f"Error writing to serial port: {e}")
    finally:
        ser.close()

# Example usage:
write_char_to_serial('A')