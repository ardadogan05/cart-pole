import numpy as np
import matplotlib.pyplot as plt

from src.python.kalman import kalman_step
from src.python.cartpole import dynamics
from src.python.lqr import K


u_max = 10 # Max force for realism
dt = 0.01
position_std = 0.002          # 2 mm
angle_std = np.deg2rad(0.2)   # 0.2 deg
x_hat = np.zeros(4)
P = np.eye(4)
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
x_hat_list = np.zeros((N, 4))

u_prev = 0.0

for i in range(N):

    y = np.array([
            x[0] + np.random.normal(0, position_std),
            x[2] + np.random.normal(0, angle_std)
        ])

    x_hat, P = kalman_step(x_hat, P, u_prev, y)

    u = float(-K @ x_hat)
    u = np.clip(u, -u_max, u_max)


    x_list[i] = x
    x_hat_list[i] = x_hat
    y_list[i] = y
    t_list[i] = t
    u_list[i] = u

    x_dot = dynamics(x, u)
    x += x_dot * dt
    t += dt
    u_prev = u

fig, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

# Position
axs[0].plot(t_list, x_list[:, 0], label="True")
axs[0].plot(t_list, y_list[:, 0], label="Measured", alpha=0.5)
axs[0].plot(t_list, x_hat_list[:, 0], label="Estimated")
axs[0].set_ylabel("Position [m]")
axs[0].legend()

# Cart velocity
axs[1].plot(t_list, x_list[:, 1], label="True")
axs[1].plot(t_list, x_hat_list[:, 1], label="Estimated")
axs[1].set_ylabel("Velocity [m/s]")
axs[1].legend()

# Pole angle
axs[2].plot(t_list, np.rad2deg(x_list[:, 2]), label="True")
axs[2].plot(t_list, np.rad2deg(y_list[:, 1]), label="Measured", alpha=0.5)
axs[2].plot(t_list, np.rad2deg(x_hat_list[:, 2]), label="Estimated")
axs[2].set_ylabel("Angle [deg]")
axs[2].legend()

# Angular velocity
axs[3].plot(t_list, np.rad2deg(x_list[:, 3]), label="True")
axs[3].plot(t_list, np.rad2deg(x_hat_list[:, 3]), label="Estimated")
axs[3].set_ylabel("Angular velocity [deg/s]")
axs[3].set_xlabel("Time [s]")
axs[3].legend()

plt.tight_layout()
plt.show()