# %% Import libraries
import numpy as np
from scipy.integrate import solve_ivp

# %% Simulation NEW
class Closed_loop:
    def __init__(self, system, controller):
        self.system = system
        self.controller = controller

    def simulate(self, time, dt, ref, init, control_index=2):
        """
        - control_index: index in the state vector that the controller observes (e.g., 2 for theta in inverted pendulum)

        Returns:
        - time: time array
        - states: np.array of shape (len(time), len(state_vector))
        """
        state = np.array(init, dtype=float)
        state_dim = len(init)
        states = []

        if np.isscalar(ref):
            ref = np.full_like(time, ref)

        for i, t in enumerate(time):
            mea = state[control_index]
            u = self.controller.control(mea, ref[i], dt)

            sol = solve_ivp(self.system.ODE, [t, t + dt], state, args=(u,))
            state = sol.y[:, -1]
            states.append(state)
        
        self.controller.reset() 

        return time, np.array(states)

    def simulate_extra(self, time, dt, ref, init, control_index=2, extra_args_func=None):

        state = np.array(init, dtype=float)
        states = []

        # Expand scalar reference to array
        if np.isscalar(ref):
            ref = np.full_like(time, ref)

        for i, t in enumerate(time):
            mea = state[control_index]
            u = self.controller.control(mea, ref[i], dt)

            # Extra args (like TL, etc.)
            # extra_args = extra_args_func[i] if extra_args_func else ()
            extra_args = extra_args_func[i]
            sol = solve_ivp(self.system.ODE, [t, t + dt], state, args=(u, extra_args))
            state = sol.y[:, -1]
            states.append(state)

        self.controller.reset()
        return time, np.array(states)

# %%
