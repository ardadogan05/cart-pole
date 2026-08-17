import numpy as np
from scipy.linalg import solve_continuous_are

from python.linear_model import A, B

Q = np.diag([1.0, 0.1, 100.0, 1.0])
R = np.array([[0.1]])

P = solve_continuous_are(A, B, Q, R)

K = np.linalg.inv(R) @ B.T @ P