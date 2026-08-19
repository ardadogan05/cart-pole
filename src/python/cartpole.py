import numpy as np

m = 0.1
M = 1.0
g = 9.81
l = 0.5

#Takes in x and u and returns x_dot
def dynamics(x,u):
    p, p_dot, theta, theta_dot = x

    p_dotdot = (u - m * g * np.sin(theta) * np.cos(theta) + m * l * theta_dot**2 * np.sin(theta))/(M + m - m * np.cos(theta)**2)
    theta_dotdot = (g * np.sin(theta) - p_dotdot * np.cos(theta))/l

    return np.array([
        p_dot,
        p_dotdot,
        theta_dot,
        theta_dotdot
    ])





