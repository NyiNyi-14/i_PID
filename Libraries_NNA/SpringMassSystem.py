# %%
class SpringMassSystem:
    def __init__(self, m, k):
        self.m = m
        self.k = k

    def ODE(self, t, state, u):
        x, v = state
        dxdt = v
        dvdt = (u - self.k * x) / self.m

        return [dxdt, dvdt]

# %%
