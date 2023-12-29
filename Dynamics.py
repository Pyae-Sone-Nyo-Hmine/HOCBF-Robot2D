import numpy as np

def dynamics(t, x, u):
    dx = np.zeros(4)
    dx[0] = x[3] * np.cos(x[2])
    dx[1] = x[3] * np.sin(x[2])
    dx[2] = u[0]
    dx[3] = u[1]
    return dx