import serial
import time

PORT = "COM3"       # change this
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

# ESP32 often resets when serial connection opens.
time.sleep(2)

#emptying after sleep to fix unicodedecodeError
ser.reset_input_buffer()

position = 0.123
angle = 0.045

#Simply writing to esp32 and receiving a response to test that communication works
for i in range(100):
    message = f"MEAS,{position},{angle}\n"

    print("TX:", message.strip())

    ser.write(message.encode())

    response = ser.readline().decode().strip()

    print("RX:", response)

ser.close()