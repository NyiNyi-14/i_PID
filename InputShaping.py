# %% Import libraries
import numpy as np

# %% Input shaping
class InputShaping:
    def __init__(self, final, initial):
        # self.t_swich = t_switch
        self.final = final
        self.initial = initial

    def delay(self, t, t_switch):
        return self.initial + (self.final - self.initial) * np.heaviside(t - t_switch, 1)

# %%
