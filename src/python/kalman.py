import numpy as np
from scipy.signal import cont2discrete

from src.python.linear_model import A, B

dt = 0.01
#from simulated model
position_std = 0.002
angle_std = np.deg2rad(0.2)

#we measure p and theta
C = np.array([
    [1,0,0,0],
    [0,0,1,0]
])

D = np.zeros((2,1))

#Discretization
Ad, Bd, Cd, Dd, _ = cont2discrete((A,B,C,D), dt)

Rk = np.diag([
    position_std**2,
    angle_std**2
])

#I trust my mathematical model, but I use a linearized version, so a small nonzero value will do
Qk = np.diag([
    1e-6,   # p
    1e-4,   # p_dot
    1e-6,   # theta
    1e-4    # theta_dot
])

#initial x and covariance
x_hat = np.zeros(4)
P = np.eye(4)

#predictions
def kalman_step(x_hat, P, u, y):
    # Prediction
    x_hat_pred = Ad @ x_hat + Bd.flatten() * u
    P_pred = Ad @ P @ Ad.T + Qk

    residual = y - C @ x_hat_pred
    S = C @ P_pred @ C.T + Rk

    Kk = P_pred @ C.T @ np.linalg.inv(S)
    x_hat = x_hat_pred + Kk @ residual

    P = (np.eye(4) - Kk @ C) @ P_pred

    return x_hat, P
