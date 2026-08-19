import numpy as np

M = 1.0
m = 0.1
g = 9.81
l = 0.5

A = np.array([
    [0, 1, 0, 0],
    [0, 0, -m*g/M, 0],
    [0, 0, 0, 1],
    [0, 0, (M+m)*g/(M*l), 0]
])

B = np.array([
    [0],
    [1/M],
    [0],
    [-1/(M*l)]
])

C_cont = np.hstack([
    B,
    A @ B,
    A @ A @ B,
    A @ A @ A @ B
])
if __name__ == "__main__":
    rank = np.linalg.matrix_rank(C_cont)

    print("A =\n", A)
    print("\nB =\n", B)
    print("\nControllability matrix =\n", C_cont)
    print("\nRank =", rank)