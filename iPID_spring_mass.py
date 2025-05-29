#%% Import Libraries
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.signal import find_peaks
import pandas as pd

from iPID_controller import iPID_controller
from Closed_loop import Closed_loop
from SpringMassSystem import SpringMassSystem
from PerformanceMetrics import PerformanceMetrics
from InputShaping import InputShaping

# %% Simulation Setup and Input
# Characteristics of the system
m = 1.0  # mass (kg)
k = 1.0  # spring constant (N/m)
spring = SpringMassSystem(m, k)

# Characteristics of the controller
# Ku = 1.0
# T = 4.4
# Kp = .6*Ku
# Ki = (2*Kp)/T
# Kd = (Kp*T)/8

# Kp = 0.5
# Ki = 0.1
# Kd = 0.05

Kp = 5
Ki = 3
Kd = 1

alpha = 1.0
iPID = iPID_controller(Kp, Ki, Kd, alpha = alpha)

# Duration and time step of simulation
duration = 100
dt = 0.01
time = np.arange(0, duration, dt)

# Stepping Reference signal
step_ref = np.ones_like(time)

# Exponential Reference signal
exp_ref = 1 - np.exp(-0.5*time)

# Initial position and velocity
x0 = 0
v0 = 0

# %%
simulation = Closed_loop(spring, iPID)
t, step_states = simulation.simulate(time, dt, ref = step_ref, init = [x0, v0], control_index=0)
plt.figure(figsize=(8, 6))
plt.plot(t, step_ref,'b-', label = r'$ x_{ref} $')
plt.plot(t, step_states[:,0],'c-', label=r'$ x $')
plt.xlabel(r"$t \ / \ \mathrm{s}$", fontsize = 14)
plt.ylabel(r"$\ x \ / \ \mathrm{m}$", fontsize = 14)
plt.xlim(0, 80)
# plt.ylim(0, step_ref[-1] +.10)
plt.legend(fontsize = 14)
plt.tick_params(axis='both', labelsize=14) 
plt.grid()
plt.tight_layout()
# plt.savefig(r'\Users\lsuic\OneDrive\Desktop\Brad\VS Code\Figures\iPID_Spring_step_test.pdf', format='pdf', bbox_inches='tight')

# %%
peak_indices, _ = find_peaks(step_states[:, 0])
 
# Get the corresponding time values
peak_values = step_states[:,0][peak_indices]
peak_times = time[peak_indices]
print(f"Peak values: {peak_values}")
 
# Time difference between first two peaks
if len(peak_times) >= 2:
    t_delta = peak_times[1] - peak_times[0]
    print(f"Time between first two peaks: {t_delta:.4f} seconds")
else:
    print("Less than two peaks found.")
 
t_switch = np.pi/ (2*np.pi*(1/t_delta))
print(f"t_switch: {t_switch:.4f} seconds")
 
# %%
zeta = 0.384 # look at MATLAB Root Locus
omega_n = 3.08 # look at MATLAB Root Locus
IS = InputShaping(zeta, omega_n)
is_ref = IS.delay(time, t_switch)
plt.plot(time, is_ref, label='Input Shaping')

# %%
t, is_states = simulation.simulate(time, dt, ref=is_ref, init = [x0, v0], control_index=0)

plt.figure(figsize=(8, 6))
plt.plot(t, is_ref,'b-', label = r'$ x_{ref} $')
plt.plot(t, is_states[:,0],'c-', label=r'$ x $')
plt.xlabel(r"$t \ / \ \mathrm{s}$", fontsize = 14)
plt.ylabel(r"$\ x \ / \ \mathrm{m}$", fontsize = 14)
plt.xlim(0, 80)
# plt.ylim(0, is_ref[-1] +.10)
plt.legend(fontsize = 14)
plt.tick_params(axis='both', labelsize=14) 
plt.grid()
plt.tight_layout()
# plt.savefig(r'\Users\lsuic\OneDrive\Desktop\Brad\VS Code\Figures\iPID_Spring_is_test.pdf', format='pdf', bbox_inches='tight')

