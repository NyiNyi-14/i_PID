# %% Import Libraries
import numpy as np

# %% System
class LinearizedPendulum:
    def __init__(self, M, m, l, g):
        self.M = M
        self.m = m
        self.g = g  # gravitational acceleration
        self.l = l  # length of the pendulum

        self.A = np.array([
                [0, 1, 0, 0],
                [0, 0, -g*m/M, 0],
                [0, 0, 0, 1],
                [0, 0, -(g * (m + M)) / (l * M), 0],
                ])
        
        self.B = np.array([
                [0],
                [1/M],
                [0],
                [1/(l*M)],
                ])

    def ODE(self, t, state, u):
        x = np.array(state).reshape(-1, 1)  
        dx = self.A @ x + self.B * u
        return dx.flatten().tolist()

# %%
