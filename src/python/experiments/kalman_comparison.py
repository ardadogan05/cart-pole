import numpy as np
import serial
import time

from src.python.kalman import kalman_step

PORT = "COM3"       # change this
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

# ESP32 often resets when serial connection opens.
time.sleep(2)

#emptying after sleep to fix unicodedecodeError
ser.reset_input_buffer()


u_max = 10 # Max force for realism
dt = 0.01
position_std = 0.002          # 2 mm
angle_std = np.deg2rad(0.2)   # 0.2 deg
x_hat_py = np.zeros(4)
P_py = np.eye(4)
N = 50

x = np.array([
    0.0,
    0.0,
    np.deg2rad(5.0),
    0.0
])
t = 0


u_prev = 0.0

for i in range(N):

    #noisy measurement
    y = np.array([
        x[0] + np.random.normal(0, position_std),
        x[2] + np.random.normal(0, angle_std)
    ])

    # python kalman
    x_hat_py, P_py = kalman_step(
        x_hat_py,
        P_py,
        u_prev,
        y
    )

    # same measurement to esp32
    message = f"MEAS,{y[0]},{y[1]}\n"
    ser.write(message.encode())

    # read esp32 estimate
    response = ser.readline().decode().strip()

    # expected:
    # EST,p,pdot,theta,thetadot

    parts = response.split(",")

    x_hat_esp = np.array([
        float(parts[1]),
        float(parts[2]),
        float(parts[3]),
        float(parts[4])
    ])

    print("Python:", x_hat_py)
    print("ESP32 :", x_hat_esp)
    print("Error :", x_hat_py - x_hat_esp)
    print()
