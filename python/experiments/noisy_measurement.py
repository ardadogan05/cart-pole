import numpy as np
import matplotlib.pyplot as plt

from python.cartpole import dynamics
from python.lqr import K

u_max = 10 # Max force for realism
dt = 0.01
position_std = 0.002          # 2 mm
angle_std = np.deg2rad(0.2)   # 0.2 deg
N = 500

x = np.array([
    0.0,
    0.0,
    np.deg2rad(5.0),
    0.0
])
t = 0


x_list = np.zeros((N,4))
t_list = np.zeros(N)
u_list = np.zeros(N)
y_list = np.zeros((N, 2))

for i in range(N):
    u = float(-K @ x)
    u = np.clip(u, -u_max, u_max)

    y = np.array([
        x[0] + np.random.normal(0, position_std),
        x[2] + np.random.normal(0, angle_std)
    ])

    x_list[i] = x
    y_list[i] = y
    t_list[i] = t
    u_list[i] = u

    dx = dynamics(x, u)
    x += dx * dt
    t += dt

theta_wrapped = (x_list[:, 2] + np.pi) % (2*np.pi) - np.pi
plt.figure()
plt.plot(t_list, x_list[:, 0], label="True")
plt.plot(t_list, y_list[:, 0], label="Measured")
plt.xlabel("Time [s]")
plt.ylabel("Cart position [m]")
plt.legend()
plt.grid()

plt.figure()
plt.plot(t_list, np.rad2deg(x_list[:, 2]), label="True")
plt.plot(t_list, np.rad2deg(y_list[:, 1]), label="Measured")
plt.xlabel("Time [s]")
plt.ylabel("Angle [deg]")
plt.legend()
plt.grid()

plt.show()