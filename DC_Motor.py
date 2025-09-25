# %% DC Motor Modelling 
class DC_motor:
    def __init__(self, R, L, Kb, Kt, J, B, TL = None):
        self.R = R
        self.L = L
        self.Kb = Kb
        self.Kt = Kt
        self.J = J
        self.B = B
        self.TL = TL
    
    def ODE(self, t, state, Va, TL):
        ia, omega = state
        ia_dt = (Va - self.R * ia - self.Kb * omega) / self.L
        omega_dt = (self.Kt * ia - self.B * omega - TL) / self.J
        return [ia_dt, omega_dt]

# %%
