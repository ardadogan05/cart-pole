import numpy as np
import serial
import time
import matplotlib.pyplot as plt

from src.python.cartpole import dynamics


PORT = "COM3"       # change this
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

# ESP32 often resets when serial connection opens.
time.sleep(2)

#emptying after sleep to fix unicodedecodeError
ser.reset_input_buffer()

dt = 0.01
position_std = 0.002          # 2 mm
angle_std = np.deg2rad(0.2)   # 0.2 deg
N = 1000

x = np.array([
    0.0,
    0.0,
    np.deg2rad(5.0),
    0.0
])

t = 0

x_hat_list = np.zeros((N, 4))
y_list = np.zeros((N, 2))
x_list = np.zeros((N,4))
t_list = np.zeros(N)
u_list = np.zeros(N)

for i in range(N):

    #noisy measurement
    y = np.array([
        x[0] + np.random.normal(0, position_std),
        x[2] + np.random.normal(0, angle_std)
    ])


    # same measurement to esp32
    message = f"MEAS,{y[0]},{y[1]}\n"
    ser.write(message.encode())

    # read esp32 estimate
    response = ser.readline().decode().strip()

    # expected:
    # DATA,p,pdot,theta,thetadot,u

    parts = response.split(",")

    #stop if esp32 response is missing or incomplete
    if len(parts) != 6 or parts[0] != "DATA":
        print("Invalid response:", response)
        break

    x_hat = np.array([
        float(parts[1]),
        float(parts[2]),
        float(parts[3]),
        float(parts[4])
    ])

    u = float(parts[5])

    x_list[i] = x
    t_list[i] = t
    u_list[i] = u
    x_hat_list[i] = x_hat
    y_list[i] = y

    x_dot = dynamics(x,u)
    x += x_dot * dt
    t += dt

ser.close()

plt.figure()

plt.subplot(3, 2, 1)
plt.plot(t_list, x_list[:, 0])
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.grid()

plt.subplot(3, 2, 2)
plt.plot(t_list, x_list[:, 1])
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m/s]")
plt.grid()

plt.subplot(3, 2, 3)
plt.plot(t_list, np.rad2deg(x_list[:, 2]))
plt.xlabel("Time [s]")
plt.ylabel("Angle [deg]")
plt.grid()

plt.subplot(3, 2, 4)
plt.plot(t_list, np.rad2deg(x_list[:, 3]))
plt.xlabel("Time [s]")
plt.ylabel("Angular velocity [deg/s]")
plt.grid()

plt.subplot(3, 2, 5)
plt.plot(t_list, u_list)
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.grid()

plt.tight_layout()
plt.show()

#relevant metrics
theta_deg = np.abs(np.rad2deg(x_list[:, 2]))

settling_time = None

for i in range(N):
    if np.all(theta_deg[i:] < 0.5):
        settling_time = t_list[i]
        break

if settling_time is not None:
    print("Settling time:", round(settling_time, 2), "s")
else:
    print("System did not settle")

max_position = np.max(np.abs(x_list[:, 0]))
max_angle = np.max(np.abs(np.rad2deg(x_list[:, 2])))
max_force = np.max(np.abs(u_list))

print("Max position:", round(max_position, 3), "m")
print("Max angle:", round(max_angle, 2), "deg")
print("Max force:", round(max_force, 2), "N")
