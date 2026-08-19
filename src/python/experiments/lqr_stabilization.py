import numpy as np
import matplotlib.pyplot as plt

from src.python.cartpole import dynamics
from src.python.lqr import K

u_max = 10 # Max force for realism
N = 500
dt = 0.01

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

for i in range(N):
    u = float(-K @ x)
    u = np.clip(u, -u_max, u_max) #for realizm

    dx = dynamics(x,u)
    x += dx*dt
    t += dt

    x_list[i] = x
    t_list[i] = t
    u_list[i] = u

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

plt.figure()
plt.plot(t_list, u_list)
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.grid()

plt.show()