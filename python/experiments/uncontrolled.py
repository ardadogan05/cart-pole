import matplotlib.pyplot as plt
import numpy as np

from python.cartpole import dynamics

dt = 0.01
N = 500

x = np.array([
    0.0,
    0.0,
    np.deg2rad(5.0),
    0.0
])
t = 0
u = 0.0

x_list = np.zeros((N,4))
t_list = np.zeros(N)

for i in range(N):
    dx = dynamics(x,u)
    x += dx*dt
    t += dt
    x_list[i] = x
    t_list[i] = t


theta_wrapped = (x_list[:, 2] + np.pi) % (2*np.pi) - np.pi
plt.figure()
plt.plot(t_list, x_list[:, 0])
plt.xlabel("Time [s]")
plt.ylabel("Cart position [m]")
plt.grid()

plt.figure()
plt.plot(t_list, np.rad2deg(theta_wrapped))
plt.xlabel("Time [s]")
plt.ylabel("Angle [deg]")
plt.grid()

plt.show()