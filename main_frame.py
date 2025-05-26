# %%
import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd

from PID_controller import PID_controller
from Closed_loop import Closed_loop
from LinearizedPendulum import LinearizedPendulum
from PerformanceMetrics import PerformanceMetrics
from sampling import sampling

# %%
print(os.getcwd())
os.chdir("/Users/nyinyia/Documents/09_LSU_GIT/i_PID_branch/folder")
print(os.listdir())

# %%
pendulum_para = {
    "M": (1.0, 0.0),
    "m": (0.1, 0.0),
    "l": (0.5, 0.1),
    "g": (9.81, 0.0),
}

# para_nom = DC_motor(motor_params["R"][0], 
#                  motor_params["L"][0], 
#                  motor_params["Kb"][0], 
#                  motor_params["Kt"][0], 
#                  motor_params["J"][0],
#                  motor_params["B"][0])

Ku_pen = 10
dt_pen = 1.256
Kp_pen = 0.6 * Ku_pen
Ki_pen = (2 * Kp_pen)/ dt_pen
Kd_pen = (Kp_pen * dt_pen)/ 8


dt = 0.01
duration = 10
time = np.arange(0, duration, dt)
init = [0, 0, 0.01, 0]  #  [x, dx, theta, dtheta]


# %%
ref = 180 * np.ones_like(time)
pendulum = LinearizedPendulum(M = pendulum_para["M"][0], 
                              m = pendulum_para["m"][0], 
                              l = pendulum_para["l"][0], 
                              g = pendulum_para["g"][0])

PID = PID_controller(Kp = Kp_pen, Ki = Ki_pen, Kd = Kd_pen)
closed_loop = Closed_loop(pendulum, PID)
time, states = closed_loop.simulate(time, dt, ref, init, control_index=2)

# %%
plt.plot(time, states[:, 2], label='theta')
plt.plot(time, ref * np.ones_like(time), label='ref', linestyle='--')
plt.grid()

# %%
evaluation = PerformanceMetrics(time, ref)
tr, ts, o_s, sse, ae, re = evaluation.step_response(states[:, 2])

# %%

table = {
    "Performance Evaluation": ["PID",],
    "Rise Time (s)": [tr],
    "Settling Time (s)": [ts],
    "Overshoot (%)": [o_s],
    "Steady-State Error": [sse],
    "Total Absolute Error": [ae],
    "Total Relative Error": [re],
}

data_frame = pd.DataFrame(table)

# %%
plt.figure(figsize=(8, 6))
plt.plot(time, ref, 'b-', label = r'${\theta_{\mathrm{ref}}}$', linewidth=2)
plt.plot(time, states[:, 2], 'c-', label = r'${\theta_{\mathrm{feedback}}}$', linewidth=2)
plt.xlabel(r"$t \ / \ \mathrm{s}$", fontsize = 14)
plt.ylabel(r"$\theta \ / \ \mathrm{degree}$", fontsize = 14)
# plt.title('Motion of LM Droplet', fontsize = 16)
plt.xlim(0, time[-1])
plt.ylim(0, ref[-1] + 5)
plt.tick_params(axis='both', labelsize=14) 
plt.grid(True)
plt.legend(fontsize = 12)
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/09_LSU_GIT/i_PID_branch/folder/test.pdf', format='pdf', bbox_inches='tight')

# %%
samples = 1000  # Number of samples
sample_init = sampling(pendulum_para, samples)
# para_MC = sample_init.MC_sampling() # Standard MC
para_LHS = sample_init.LHS_sampling() # LHS
# para_IM = sample_init.IM_sampling() # IM

# %%
# Plot distributions for each sampling method   
# sampling.plot_distributions(para_MC, "Monte Carlo")
sampling.plot_distributions(para_LHS, "Latin Hypercube")
# sampling.plot_distributions(para_IM, "Importance Sampling")

# %%
state_stored = []
theta_store = np.zeros((samples, len(time)))

for i in range(samples):
    l_i = para_LHS["l"][i]  # Use only varying l

    pen_LHS = LinearizedPendulum(M = pendulum_para["M"][0], 
                              m = pendulum_para["m"][0], 
                              l = l_i, 
                              g = pendulum_para["g"][0])

    PID = PID_controller(Kp_pen, Ki_pen, Kd_pen)
    CL_MC = Closed_loop(pen_LHS, PID)
    time, all_states = CL_MC.simulate(time, dt, ref, init, control_index=2)
    # state_stored.append(all_states)
    theta_store[i, :] = all_states.T[2]  # Store theta values


# %%

avg, mu_025, mu_975 = evaluation.to_plot(theta_store)

plt.plot(time, ref, label='Mean of theta')
plt.plot(time, avg, label='Mean of theta')
plt.fill_between(time, mu_025, mu_975, alpha=0.7, 
                 label=r'$95\% \text{ Confidence Interval}$')


# %%
