# %% Import libraries
import numpy as np

# %% Input shaping
class InputShaping:
    def __init__(self, zeta, omega_n):
        self.zeta = zeta     
        self.omega_n = omega_n 
        self._compute_coeff()

    def _compute_coeff(self):
        exp_term = (self.zeta * np.pi) / np.sqrt(1 - self.zeta**2)
        self.A0 = np.exp(exp_term) / (1 + np.exp(exp_term))
        self.A1 = 1 - self.A0
    
    def delay(self, t, t_switch=0):
        return self.A0 + self.A1 * np.heaviside(t - t_switch, 1)
    
    def robust_is(self, t, t_switch=0):
        # return (self.A0 + self.A1 * np.heaviside(t - t_switch, 1))**2
        return (self.A0**2 + 2 * self.A0 * self.A1 * np.heaviside(t - t_switch, 1) + 
                 self.A1**2 * np.heaviside(t - 2* t_switch, 1))

# %%
